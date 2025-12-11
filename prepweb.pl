#!/usr/bin/perl
# take clean html in webpage.html and insert into web.c IN-PLACE.

use strict;
use warnings;

# name of file with html, to edit and a tmp file
my $htmlfn = "webpage.html";
my $editfn = "web.c";
my $tmpfn = ".web.c";

# editfn must be writable
my ($dev,$ino,$mode,$xxx) = stat ($editfn);
defined($mode) or die "$editfn: $!\n";
($mode & 0200) or die "$editfn must be writable\n";

# open edit file
open EF, "<", $editfn or die "$editfn: $!\n";

# open html file
open HF, "<", $htmlfn or die "$htmlfn: $!\n";

# create temp file
open TF, ">", $tmpfn or die "$tmpfn: $!\n";

# copy editfn to tmpfn up through the first magic line
while (<EF>) {
    print TF;
    last if (/DO NOT EDIT THIS LINE 1/);
}

# now copy the html into tmpfn, making it appear in a char array
print TF "static const char page[] = \"\"\n";
while(<HF>) {
    chomp();
    $_ =~ s/\\/\\\\/g;		# retain all \ by turning into \\
    $_ =~ s/"/\\"/g;		# retain all " by turning into \"
    print TF "\"$_\\n\"\n";
}
print TF ";\n";

# skip editfn down to second magic line, again inclusive
while (<EF>) {
    if (/DO NOT EDIT THIS LINE 2/) {
	print TF;
	last;
    }
}

# copy remainder of editfn to tmpfn
while (<EF>) {
    print TF;
}

# close all files and replace edit with tmp
close HF;
close EF;
close TF;
unlink ($editfn);
rename ($tmpfn, $editfn);
