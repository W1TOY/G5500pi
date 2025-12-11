/* stand-alone main() for a program that controls a Yaesu G5500 on RPi with the same socket protocol as
 * hamlib's rotctld and/or with a web interface.
 * 
 * we support the following rotctld socket commands and their variants:
 *
 *    +\get_pos
 *    +\set_pos
 *    +\move
 *    +\park
 *    +\stop
 *    +\get_info
 *    +\dump_caps
 *
 * we support the following REST web commands or direct without leading /:
 *
 *    /get_pos
 *    /get_setpos
 *    /set_pos?az=x&el=y
 *    /move?direction=[up,down,left,right]
 *    /park
 *    /stop
 *    /get_info
 *    /dump_caps
 *    /help
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <arpa/inet.h>


// define simulation default level depending on platform
#include "isapi.h"
#if defined(ISA_PI)
#define DEF_SIM 0
#else
#define DEF_SIM 3
#endif


// max number of each client
#define MAX_ROTCLIENTS          1       // no way for more to know commanded pos of others
#define MAX_WEBCLIENTS          5       // no problem because all can use get_setpos

// glue
#include "version.h"
#include "g5500_sa.h"


// rotctld default listening port, same as rotctld
#define DEF_ROTPORT 4533
static int tcp_rotport = DEF_ROTPORT;


// web default listening port
#define DEF_WEBPORT 8008
static int tcp_webport = DEF_WEBPORT;


// capture pointer to the persistent capabilities structure
static struct rot_caps *g5500_rot_caps;

// persistent ROT structure
static ROT my_rot;


// command line options
int verbose = RIG_DEBUG_ERR;
static int sim_level = DEF_SIM;

// last set_pos
static float setpos_x, setpos_y;


/* show usage and optional message and exit(1)
 */
static void usage (const char *me, const char *errfmt, ...)
{
        char *slash = strrchr (me, '/');
        if (slash)
            me = slash + 1;

        if (errfmt) {
            va_list ap;
            va_start (ap, errfmt);
            fprintf (stderr, "Usage error: ");
            vfprintf (stderr, errfmt, ap);
            va_end (ap);
            if (!strchr(errfmt, '\n'))
                fprintf (stderr, "\n");
        }

        // usage
        fprintf (stderr, "Purpose: provide rotctld and web control for Yaesu G5500 on Rasp Pi\n");
        fprintf (stderr, "Usage: %s [options]\n", me);
        fprintf (stderr, "options:\n");
        fprintf (stderr, "  -V   : display version and exit\n");
        fprintf (stderr, "  -r p : listen on port p for rotctld commands; default %d\n", DEF_ROTPORT);
        fprintf (stderr, "  -s s : simulation level: 0=real 1=az-only 2=az+el90 3=az+el180; default %d\n", DEF_SIM);
        fprintf (stderr, "  -v   : verbose level, cummulative\n");
        fprintf (stderr, "  -w p : listen on port p for web commands; default %d\n", DEF_WEBPORT);

        // done
        exit(1);
}

/* crack args, exit if trouble
 */
static void crackArgs (int ac, char *av[])
{
        char *me = av[0];

        // crack args
        while (--ac && **++av == '-') {
            char *s = *av;
            while (*++s) {
                switch (*s) {
                case 'V':
                    printf ("Version %s\n", VERSION);
                    exit(0);
                    break;
                case 'r':
                    if (ac < 2)
                        usage (me, "-r requires rotctld port");
                    tcp_rotport = atoi (*++av);
                    if (tcp_rotport < 1000 || tcp_rotport > 65535)
                        usage (me, "port must be 1000 .. 65535");
                    ac--;
                    break;
                case 's':
                    if (ac < 2)
                        usage (me, "-s requires sim level");
                    sim_level = atoi (*++av);
                    ac--;
                    break;
                case 'v':
                    verbose++;
                    break;
                case 'w':
                    if (ac < 2)
                        usage (me, "-r requires web port");
                    tcp_webport = atoi (*++av);
                    if (tcp_webport < 1000 || tcp_webport > 65535)
                        usage (me, "port must be 1000 .. 65535");
                    ac--;
                    break;
                default:
                    usage (me, "Unknown option");
                    break;
                }
            }
        }

        if (ac > 0)
            usage (me, "Unexpected argument");
}

/* set up a server socket on the given port.
 * return socket else exit.
 */
static int prepareServer(int port)
{
        struct sockaddr_in serv_socket;
        int socket_fd;
        int reuse = 1;

        // create socket endpoint
        if ((socket_fd = socket (AF_INET, SOCK_STREAM, 0)) < 0) {
            rig_debug (RIG_DEBUG_ERR, "setsockopt(SO_REUSEADDR): %s\n", strerror(errno));
            exit(1);
        }
        rig_debug (RIG_DEBUG_VERBOSE, "new socket %d ok\n", socket_fd);

        // bind to given port for any IP address
        memset (&serv_socket, 0, sizeof(serv_socket));
        serv_socket.sin_family = AF_INET;
        serv_socket.sin_addr.s_addr = htonl (INADDR_ANY);
        serv_socket.sin_port = htons ((unsigned short)port);
        if (setsockopt (socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
            rig_debug (RIG_DEBUG_ERR, "setsockopt(SO_REUSEADDR): %s\n", strerror(errno));
            exit(1);
        }
        if (bind(socket_fd, (struct sockaddr*)&serv_socket, sizeof(serv_socket)) < 0) {
            rig_debug (RIG_DEBUG_ERR, "bind to port %d: %s\n", port, strerror(errno));
            exit(1);
        }
        rig_debug (RIG_DEBUG_VERBOSE, "bind port %d ok\n", port);

        // prepare a listening server socket
        if (listen (socket_fd, 5) < 0) {
            rig_debug (RIG_DEBUG_ERR, "listen for port %d: %s\n", port, strerror(errno));
            exit(1);
        }
        rig_debug (RIG_DEBUG_VERBOSE, "listen port %d ok on socket %d\n", port, socket_fd);

        return (socket_fd);
}

/* accept a new client on the given server socket, known to be knocking on the door.
 * return rw FILE* else exit if trouble.
 */
static FILE *acceptNewClient(int server_socket)
{
        // accept client
        struct sockaddr_in cli_socket;
        socklen_t cli_len = sizeof(cli_socket);
        int cli_fd = accept (server_socket, (struct sockaddr *)&cli_socket, &cli_len);
        if (cli_fd < 0) {
            rig_debug (RIG_DEBUG_ERR, "accept: %s\n", strerror(errno));
            exit(1);
        }
        rig_debug (RIG_DEBUG_VERBOSE, "accept ok on socket %d\n", cli_fd);

        // build FILE *
        FILE *fp = fdopen (cli_fd, "r+");
        if (!fp) {
            rig_debug (RIG_DEBUG_ERR, "fdopen failed\n");
            exit(1);
        }

        // immediate io
        setbuf (fp, NULL);

        // return
        return (fp);
}

/* invoke rot_register
 */
static void captureCapabilities()
{
        // calls rot_register
        (void) DECLARE_INITROT_BACKEND_dummy();
}

/* round-about means to capture the backend struct rot_caps
 */
void rot_register (struct rot_caps *rc)
{
        g5500_rot_caps = rc;
}

/* debug message
 */
void rig_debug (int level, const char *fmt, ...)
{
        if (level <= verbose) {
            va_list ap;
            va_start (ap, fmt);
            vfprintf (stdout, fmt, ap);
            va_end (ap);
        }
}

/* return 0 if punctuation character p is one of the legal prefix command characters, else -1
 */
static int punctOk (char p)
{
        return (p == '+' || p == ';' || p == '|' || p == ',' ? 0 : -1);
}


/* run another rot command read from fp.
 * fclose fp and return -1 if io trouble, else leave open and return 0.
 * N.B. protocol errors do NOT return -1.
 *
 * There are many protocol variations that seem to divide into four categories:
 *
 *   single letter: get reply is one lone item per line; set reply is just RPRT #
 *   \long name   : same as single letter
 *   +\long name  : get reply is echo:, key:value pairs, RPRT; set reply is echo params, RPRT #
 *   x\long name  : get reply is one line of echo: paramsx RPRT #
 *
 * three examples:
 *
 *       p
 *       10.00
 *       20.00
 *
 *       \get_pos
 *       10.00
 *       20.00
 *
 *       +\get_pos
 *       get_pos:
 *       Azimuth: 20.00
 *       Elevation: 30.00
 *       RPRT 0
 *
 *       ;\get_pos              # or |\ or ,\
 *       get_pos:;Azimuth: 20.00;Elevation: 30.00;RPRT 0
 *
 *
 *
 *       P 20 30
 *       RPRT 0
 *
 *       P 1000 2000
 *       RPRT -1
 *
 *       \set_pos 20 30
 *       RPRT 0
 *
 *       +\set_pos 20 30
 *       set_pos: 20 30
 *       RPRT 0
 *
 *       +\set_pos 1000 2000
 *       set_pos: 1000 2000
 *       RPRT -1
 *
 *       ;\set_pos 20 30        # or |\ or ,\
 *       set_pos: 20 30;RPRT 0
 *
 *
 *
 *       _
 *       Dummy rotator
 *
 *       \get_info
 *       Dummy rotator
 *
 *       +\get_info
 *       get_info:
 *       Info: Dummy rotator
 *       RPRT 0
 *
 *       ;\get_info             # or |\ or ,\
 *       get_info:;Info: Dummy rotator;RPRT 0
 *
 *
 */
static int runRotator (FILE *fp)
{
        char buf[100];
        int a, b;
        float x, y;
        int err;
        char p;
#define MOD_FOR_GPREDICT

#ifndef MOD_FOR_GPREDICT
        // read command
        if (!fgets (buf, sizeof(buf), fp)) {
            fclose (fp);
            return (-1);
        }
        rig_debug (RIG_DEBUG_VERBOSE, "client %d message: %s", fileno(fp), buf);

#else
        // read command
        errno = 0;
        int i = 0;
        while (i < sizeof(buf) - 1) {
            int c = fgetc(fp);
//            printf("read one character = %d \n", c);
            if (c == 112 || c == 83) { // 'p' or 'S' for rotctld get_pos or stop
                if (i==0) buf[i++] = (char)c; // somehow gpredict seems to send p at the end
                buf[i++] = (char)'\n';
                break;
           }
            else if (c == EOF) {
                if (i == 0) {
                    if (feof(fp))
                        printf("runRotator: Client %d EOF\n", fileno(fp));
                    else
                        printf("runRotator: Client %d Error: %s\n", fileno(fp), strerror(errno));
                    fclose(fp);
                    return -1;
                }
                break;
            }
            if (c == '\n' || c == '\r') break;
            buf[i++] = (char)c;
        }
        buf[i] = '\0';

//        printf("client %d message: %s", fileno(fp), buf); // added to see why message from gpredict doesn't show up

        rig_debug (RIG_DEBUG_VERBOSE, "client %d message: %s", fileno(fp), buf);
#endif

        // trim trailing \n
        buf[strlen(buf)-1] = '\0';
        rig_debug (RIG_DEBUG_VERBOSE, "RX: %d '%s'\n", (int)strlen(buf), buf);

        // prepare stream for writing
        fseek (fp, 0, SEEK_CUR);

        // crack, send reply

        // get_pos, p

        if (strcmp (buf, "p") == 0 || strcmp (buf, "\\get_pos") == 0) {
            // default protocol
            err = (*g5500_rot_caps->get_position) (&my_rot, &x, &y);
            if (err == RIG_OK) {
                fprintf (fp, "%g\n", x);
                fprintf (fp, "%g\n", y);
            } else {
                fprintf (fp, "RPRT %d\n", err);
            }

        } else if (strcmp (buf+1, "\\get_pos") == 0 && punctOk (buf[0]) == 0) {
            // extended protocol
            err = (*g5500_rot_caps->get_position) (&my_rot, &x, &y);
            p = buf[0];
            if (p == '+')
                p = '\n';
            if (err != RIG_OK)
                x = y = 0;
            fprintf (fp, "get_pos:%cAzimuth: %g%cElevation: %g%cRPRT %d\n", p, x, p, y, p, err);



        // set_pos, P

        } else if (sscanf (buf, "P %g %g", &x, &y) == 2
                                    || sscanf (buf, "\\set_pos %g %g", &x, &y) == 2) {
            // default protocol
            err = (*g5500_rot_caps->set_position) (&my_rot, x, y);
            fprintf (fp, "RPRT %d\n", err);
            if (err == RIG_OK) {
                setpos_x = x;
                setpos_y = y;
            }

        } else if (sscanf(buf, "%c\\set_pos %g %g", &p, &x, &y) == 3 && punctOk(p) == 0) {
            // extended protocol
            err = (*g5500_rot_caps->set_position) (&my_rot, x, y);
            if (p == '+')
                p = '\n';
            fprintf (fp, "set_pos: %g %g%cRPRT %d\n", x, y, p, err);
            if (err == RIG_OK) {
                setpos_x = x;
                setpos_y = y;
            }



        // move, M

        } else if (sscanf (buf, "M %d %d", &a, &b) == 2
                                    || sscanf (buf, "\\move %d %d", &a, &b) == 2) {
            // default protocol
            err = (*g5500_rot_caps->move) (&my_rot, a, b);
            fprintf (fp, "RPRT %d\n", err);
        } else if (sscanf (buf, "%c\\move %d %d", &p, &a, &b) == 3 && punctOk (p) == 0) {
            // extended protocol
            err = (*g5500_rot_caps->move) (&my_rot, a, b);
            if (p == '+')
                p = '\n';
            fprintf (fp, "move: %d %d%cRPRT %d\n", a, b, p, err);



        // park, K

        } else if (strcmp (buf, "K") == 0 || strcmp (buf, "\\park") == 0) {
            // default protocol
            err = (*g5500_rot_caps->park) (&my_rot);
            fprintf (fp, "RPRT %d\n", err);
        } else if (strcmp (buf+1, "\\park") == 0 && punctOk (buf[0]) == 0) {
            // extended protocol
            err = (*g5500_rot_caps->park) (&my_rot);
            p = buf[0];
            if (p == '+')
                p = '\n';
            fprintf (fp, "park:%cRPRT %d\n", p, err);



        // stop, S

        } else if (strcmp (buf, "S") == 0 || strcmp (buf, "\\stop") == 0) {
            // default protocol
            err = (*g5500_rot_caps->stop) (&my_rot);
            fprintf (fp, "RPRT %d\n", err);
        } else if (strcmp (buf+1, "\\stop") == 0 && punctOk (buf[0]) == 0) {
            // extended protocol
            err = (*g5500_rot_caps->stop) (&my_rot);
            p = buf[0];
            if (p == '+')
                p = '\n';
            fprintf (fp, "stop:%cRPRT %d\n", p, err);



        // get_info, _

        } else if (strcmp (buf, "_") == 0 || strcmp (buf, "\\get_info") == 0) {
            // default protocol
            fprintf (fp, "%s\n", (*g5500_rot_caps->get_info) (&my_rot));
        } else if (strcmp (buf+1, "\\get_info") == 0 && punctOk (buf[0]) == 0) {
            // extended protocol
            p = buf[0];
            if (p == '+')
                p = '\n';
            fprintf (fp, "get_info:%cInfo: %s%cRPRT 0\n", p, (*g5500_rot_caps->get_info)(&my_rot), p);



        // dump_caps, 1   -- does not follow standard protocol

        } else if (strcmp (buf, "1") == 0 || strcmp (buf, "\\dump_caps") == 0
                        || (strcmp (buf+1, "\\dump_caps") == 0 && punctOk (buf[0]) == 0)) {
            fprintf (fp, "Min Azimuth: %g\n", g5500_rot_caps->min_az);
            fprintf (fp, "Max Azimuth: %g\n", g5500_rot_caps->max_az);
            fprintf (fp, "Min Elevation: %g\n", g5500_rot_caps->min_el);
            fprintf (fp, "Max Elevation: %g\n", g5500_rot_caps->max_el);
            fprintf (fp, "RPRT 0\n");


        // dump_state, 2

        } else if (strcmp (buf, "2") == 0 || strcmp (buf, "\\dump_state") == 0
                        || (strcmp (buf+1, "\\dump_state") == 0 && punctOk (buf[0]) == 0)) {
            float az = 0, el = 0;
            (*g5500_rot_caps->get_position) (&my_rot, &az, &el);
            fprintf (fp, "Azimuth: %g\n", az);
            fprintf (fp, "Elevation: %g\n", el);
            fprintf (fp, "Min Azimuth: %g\n", g5500_rot_caps->min_az);
            fprintf (fp, "Max Azimuth: %g\n", g5500_rot_caps->max_az);
            fprintf (fp, "Min Elevation: %g\n", g5500_rot_caps->min_el);
            fprintf (fp, "Max Elevation: %g\n", g5500_rot_caps->max_el);
            fprintf (fp, "RPRT 0\n");

        // unrecognized

        } else {
            fprintf (fp, "RPRT %d\n", -RIG_EINVAL);
        }

        // prepare stream for reading
        fseek (fp, 0, SEEK_CUR);

        // check for io error, else ok
        if (feof(fp) || ferror(fp)) {
            fclose (fp);
            return (-1);
        } else {
            // no io errors
            return (0);
        }
}

/* send the http preamble for plain text content
 */
static void startPlainTextHTTP(FILE *fp)
{
        fprintf (fp, "HTTP/1.0 200 OK\r\n");
        fprintf (fp, "User-Agent: g5500_sa\r\n");
        fprintf (fp, "Content-Type: text/plain; charset=us-ascii\r\n");
        fprintf (fp, "Connection: close\r\n");
        fprintf (fp, "\r\n");
}

/* run one web or direct command known to be pending on fp.
 * web: always fclose after replying and return -1.
 * direct: return -1 if io trouble else leave open and return 0.
 */
static int runWeb (FILE *fp)
{
        char buf[256];
//        char move_dir[10];
        char move_dir[11];
        char *cmd;
        int is_http = 0;
        float x, y;

        // read first line
        if (!fgets (buf, sizeof(buf), fp)) {
            fclose (fp);
            return (-1);
        }
        rig_debug (RIG_DEBUG_VERBOSE, "client %d message: %s", fileno(fp), buf);

        // decide whether HTTP or direct
        if (strncmp (buf, "GET /", 5) == 0 && strstr (buf, "HTTP")) {

            // looks like http
            char tmp[256];
            is_http = 1;

            // command starts after the slash
            cmd = buf + 5;

            // read and discard through first blank line
            while (fgets (tmp, sizeof(tmp), fp)) {
                rig_debug (RIG_DEBUG_VERBOSE, "client %d: %s", fileno(fp), tmp);
                if (tmp[0] == '\n' || tmp[0] == '\r')
                    break;
            }

        } else {

            // direct
            cmd = buf;
        }

        // replace first whitespace with '\0';
        cmd[strcspn (cmd, " \r\n")] = '\0';

        // prep stream for writing
        fseek (fp, 0, SEEK_CUR);


        // crack and respond

        if (strcmp (cmd, "get_pos") == 0) {

            if (is_http)
                startPlainTextHTTP(fp);
            int err = (*g5500_rot_caps->get_position) (&my_rot, &x, &y);
            if (err == RIG_OK)
                fprintf (fp, "%g %g\n", x, y);
            else
                fprintf (fp, "err: can not get position, code %d\n", err);

        } else if (sscanf (cmd, "set_pos?az=%f&el=%f", &x, &y) == 2) {

            if (is_http)
                startPlainTextHTTP(fp);
            int err = (*g5500_rot_caps->set_position) (&my_rot, x, y);
            if (err == RIG_OK) {
                fprintf (fp, "ok\n");
                setpos_x = x;
                setpos_y = y;
            } else
                fprintf (fp, "err: can not set position, code %d\n", err);

        } else if (sscanf (cmd, "move?direction=%10s", move_dir) == 1) {

            if (is_http)
                startPlainTextHTTP(fp);
            int dir = 999;
            if (strcmp (move_dir, "up") == 0)
                dir = ROT_MOVE_UP;
            else if (strcmp (move_dir, "down") == 0)
                dir = ROT_MOVE_DOWN;
            else if (strcmp (move_dir, "left") == 0)
                dir = ROT_MOVE_LEFT;
            else if (strcmp (move_dir, "right") == 0)
                dir = ROT_MOVE_RIGHT;

            if (dir == 999) {
                fprintf (fp, "err: unknown direction\n");
            } else {
                int err = (*g5500_rot_caps->move) (&my_rot, dir, 0);
                if (err == RIG_OK)
                    fprintf (fp, "ok\n");
                else
                    fprintf (fp, "err: error moving %s, code %d\n", move_dir, err);
            }

        } else if (strcmp (cmd, "get_setpos") == 0) {

            if (is_http)
                startPlainTextHTTP(fp);
            fprintf (fp, "%g %g\n", setpos_x, setpos_y);

        } else if (strcmp (cmd, "park") == 0) {

            if (is_http)
                startPlainTextHTTP(fp);
            int err = (*g5500_rot_caps->park) (&my_rot);
            if (err == RIG_OK) {
                fprintf (fp, "ok\n");
                setpos_x = 0;
                setpos_y = 0;
            } else
                fprintf (fp, "err: error parking, code %d\n", err);

        } else if (strcmp (cmd, "stop") == 0) {

            if (is_http)
                startPlainTextHTTP(fp);
            int err = (*g5500_rot_caps->stop) (&my_rot);
            if (err == RIG_OK)
                fprintf (fp, "ok\n");
            else
                fprintf (fp, "err: error stopping, code %d\n", err);

        } else if (strcmp (cmd, "get_info") == 0) {

            if (is_http)
                startPlainTextHTTP(fp);
            fprintf (fp, "%s\n", (*g5500_rot_caps->get_info) (&my_rot));

        } else if (strcmp (cmd, "dump_caps") == 0) {

            if (is_http)
                startPlainTextHTTP(fp);
            fprintf (fp, "Azimuth %g .. %g Elevation %g .. %g\n",
                        g5500_rot_caps->min_az, g5500_rot_caps->max_az,
                        g5500_rot_caps->min_el, g5500_rot_caps->max_el);

        } else if (strcmp (cmd, "help") == 0) {

            if (is_http)
                startPlainTextHTTP(fp);
            fprintf (fp, "Available commands:\n");
            fprintf (fp, "    get_pos\n");
            fprintf (fp, "    get_setpos\n");
            fprintf (fp, "    set_pos?az=x&el=y\n");
            fprintf (fp, "    move?direction=[up,down,left,right]\n");
            fprintf (fp, "    park\n");
            fprintf (fp, "    stop\n");
            fprintf (fp, "    get_info\n");
            fprintf (fp, "    dump_caps\n");


        } else if (strcmp (cmd, "index.html") == 0 || strlen(cmd) == 0) {

            if (sendWebPage(fp) < 0)
                fprintf (fp, "err: can not send web page\n");

        } else {

            if (is_http)
                startPlainTextHTTP(fp);
            fprintf (fp, "err: unrecognized command\n");
        }

        // web always closes, direct only if io trouble
        if (is_http || feof(fp) || ferror(fp)) {
            fclose (fp);
            return (-1);
        } else {
            return (0);
        }
}


/* call rotator's init once and set sim level
 */
static void initRotator()
{
        int err;

        // init
        err = (*g5500_rot_caps->rot_init) (&my_rot);
        if (err != RIG_OK) {
            rig_debug (RIG_DEBUG_ERR, "init failed: %d\n", err);
            exit(1);
        }

        // find simulation paramter token
        for (const struct confparams *cp = g5500_rot_caps->cfgparams; cp->token != RIG_CONF_END; cp++) {
            if (strcmp (cp->name, "simulator") == 0) {
                if (sim_level < cp->u.n.min || sim_level > cp->u.n.max) {
                    rig_debug (RIG_DEBUG_ERR, "sim level %d must be %g .. %g\n", sim_level,
                                                                cp->u.n.min, cp->u.n.max);
                    exit(1);
                } else {
                    char strval[20];
                    snprintf (strval, sizeof(strval), "%d", sim_level);
                    err = (*g5500_rot_caps->set_conf)(&my_rot, cp->token, strval);
                    if (err != RIG_OK) {
                        rig_debug (RIG_DEBUG_ERR, "sim level %d failed\n", sim_level);
                        exit (1);
                    }
                    break;
                }
            }
        }
}

static void setSignal (int signo, void (*handler)(int))
{
        struct sigaction sa;

        sa.sa_handler = handler;
        sigemptyset(&sa.sa_mask);
        sigaddset(&sa.sa_mask, signo);  // defer this sig while in progress
        sa.sa_flags = SA_RESTART;       // restart blocking sys calls, defer additional sigs

        if (sigaction (signo, &sa, NULL) < 0) {
            rig_debug (RIG_DEBUG_ERR, "sigaction(%d): %s\n", signo, strerror(errno));
            exit(1);
        }
}

/* increment and roll over the verbose level on receipt of SIGUSR1
 */
static void onSU1 (int unused)
{
        (void) unused;

        if (++verbose > RIG_DEBUG_TRACE)
            verbose = RIG_DEBUG_ERR;
}

/* try to stop then exit
 */
static void onAnyStopSignal (int unused)
{
        (void) unused;

        (void) (*g5500_rot_caps->stop) (&my_rot);

        // above just informs the control thread, give it time to respond
        usleep (100000);
        exit(1);
}

/* add active clients to fdsp, bumping max_fd if largest
 */
static int addClientFD (fd_set *fdsp, int max_fd, FILE *clients[], int n_clients)
{
        for (int i = 0; i < n_clients; i++) {
            FILE *fp = clients[i];
            if (fp) {
                int fd = fileno(fp);
                FD_SET (fd, fdsp);
                if (fd > max_fd)
                    max_fd = fd;
            }
        }
        return (max_fd);
}

/* call func_p for all clients marked as ready in fdsp.
 * close clients when EOF
 */
static void checkForClientMessage (fd_set *fdsp, FILE *clients[], int n_clients,
                        const char *whom, int (*func_p)(FILE *fp))
{
        for (int i = 0; i < n_clients; i++) {
            if (clients[i]) {
                int fd = fileno(clients[i]);
                if (FD_ISSET (fd, fdsp)) {
                    rig_debug (RIG_DEBUG_VERBOSE, "message from %s client %d\n", whom, fd);
                    if ((*func_p)(clients[i]) < 0) {
                        rig_debug (RIG_DEBUG_VERBOSE, "%s client %d closed\n", whom, fd);
                        clients[i] = NULL;
                    }
                }
            }
        }
}

/* add any new client waiting to connect to server to list.
 * return 0 if ok, else -1 no more room
 */
static int checkForNewClient (fd_set *fdsp, int server, FILE *clients[], int max_clients, const char *whom)
{
        if (FD_ISSET (server, fdsp)) {
            FILE *fp = acceptNewClient (server);
            for (int i = 0; i < max_clients; i++) {
                // find unused entry
                if (!clients[i]) {
                    rig_debug (RIG_DEBUG_VERBOSE, "new %s client %d\n", whom, fileno(fp));
                    clients[i] = fp;
                    return (0);
                }
            }
            fclose (fp); 
            return (-1);
        }
        return (0);
}

/* main program, see usage()
 */
int main (int ac, char *av[])
{
        // immediate stdout
        setbuf (stdout, NULL);

        // handle write errors inline
        setSignal (SIGPIPE, SIG_IGN);

        // setup
        crackArgs (ac, av);
        captureCapabilities();
        initRotator();

        // catch SIGUSR1 to increment verbose
        setSignal (SIGUSR1, onSU1);

        // stop on any of several likely signals
        setSignal (SIGINT, onAnyStopSignal);
        setSignal (SIGHUP, onAnyStopSignal);
        setSignal (SIGQUIT, onAnyStopSignal);
        setSignal (SIGTERM, onAnyStopSignal);

        // create the two persistent server sockets
        int rot_server = prepareServer(tcp_rotport);
        int web_server = prepareServer(tcp_webport);

        // collection of clients
        // N.B. be very careful mixing file descriptors and FILE *
        FILE *rot_clients[MAX_ROTCLIENTS];
        memset (rot_clients, 0, sizeof(rot_clients));
        FILE *web_clients[MAX_WEBCLIENTS];
        memset (web_clients, 0, sizeof(web_clients));

        // forever
        for(;;) {

            // prepare list of sockets to examine
            fd_set sockets;
            FD_ZERO (&sockets);
            int max_fd = 0;

            // add servers
            FD_SET (rot_server, &sockets);
            if (rot_server > max_fd)
                max_fd = rot_server;
            FD_SET (web_server, &sockets);
            if (web_server > max_fd)
                max_fd = web_server;

            // add clients
            max_fd = addClientFD (&sockets, max_fd, rot_clients, MAX_ROTCLIENTS);
            max_fd = addClientFD (&sockets, max_fd, web_clients, MAX_WEBCLIENTS);

            // wait forever
            int ns = select (max_fd+1, &sockets, NULL, NULL, NULL);
            if (ns < 0) {
                rig_debug (RIG_DEBUG_ERR, "select(): %s\n", strerror(errno));
                exit(1);
            }
            if (ns == 0) {
                rig_debug (RIG_DEBUG_ERR, "select(): timed out\n");
                exit(1);
            }
            if (ns > 0) {

                // new client?
                if (checkForNewClient (&sockets, rot_server, rot_clients, MAX_ROTCLIENTS, "rot") < 0)
                    rig_debug (RIG_DEBUG_ERR, "too many rot clients\n");
                if (checkForNewClient (&sockets, web_server, web_clients, MAX_WEBCLIENTS, "web") < 0)
                    rig_debug (RIG_DEBUG_ERR, "too many web clients\n");

                // new message?
                checkForClientMessage (&sockets, rot_clients, MAX_ROTCLIENTS, "rot", runRotator);
                checkForClientMessage (&sockets, web_clients, MAX_WEBCLIENTS, "web", runWeb);
            }
        }

        return (0);
}
