#!/bin/bash
# handy G5500 pin test
# loops forever until Control-C; enter cw, ccw, up or down to test motion, any other input or Ctrl-C stops

# Common path for all GPIO access
BASE_GPIO_PATH=/sys/class/gpio

# test if this is a pi
if ! [ -x $BASE_GPIO_PATH ] ; then
    echo not a Pi
    exit 1;
fi


# Assign names to GPIO pin numbers
PIN_CW=25           # header 22
PIN_CCW=8           # header 24
PIN_UP=7            # header 26
PIN_DOWN=1          # header 28

# states
PINSTATE_ACTIVE=1
PINSTATE_IDLE=0

# export a pin if not already exported
exportPin()
{
    if [ ! -e $BASE_GPIO_PATH/gpio$1 ]; then
        echo "$1" > $BASE_GPIO_PATH/export
    fi
}

# set a pin as an output
setAsOutput()
{
    echo "out" > $BASE_GPIO_PATH/gpio$1/direction
}

# change state of a pin
setPinState()
{
    echo $2 > $BASE_GPIO_PATH/gpio$1/value
}

# stop all motion
allStop()
{
    setPinState $PIN_CW $PINSTATE_IDLE
    setPinState $PIN_CCW $PINSTATE_IDLE
    setPinState $PIN_UP $PINSTATE_IDLE
    setPinState $PIN_DOWN $PINSTATE_IDLE
}

# Export pins so that we can use them
exportPin $PIN_CW
exportPin $PIN_CCW
exportPin $PIN_UP
exportPin $PIN_DOWN

# Set all pins as outputs
setAsOutput $PIN_CW
setAsOutput $PIN_CCW
setAsOutput $PIN_UP
setAsOutput $PIN_DOWN

# all stop
allStop

# stop and exit
onInt()
{
    allStop
    exit
}

# control-c handler
trap onInt SIGINT

# Loop forever until user types Ctrl-C
while true; do
    echo enter one of: cw ccw up down or anything else or Ctrl-C to stop
    read cmd
    case $cmd in
    cw)
        setPinState $PIN_CCW $PINSTATE_IDLE
        setPinState $PIN_CW $PINSTATE_ACTIVE
        ;;
    ccw)
        setPinState $PIN_CW $PINSTATE_IDLE
        setPinState $PIN_CCW $PINSTATE_ACTIVE
        ;;
    up)
        setPinState $PIN_DOWN $PINSTATE_IDLE
        setPinState $PIN_UP $PINSTATE_ACTIVE
        ;;
    down)
        setPinState $PIN_UP $PINSTATE_IDLE
        setPinState $PIN_DOWN $PINSTATE_ACTIVE
        ;;
    *)
        # stop with any other command
        allStop
        ;;
    esac
done
