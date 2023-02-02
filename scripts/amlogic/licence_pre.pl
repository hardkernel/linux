#!/usr/bin/perl -W
# SPDX-License-Identifier: (GPL-2.0+ OR MIT)
#
# Copyright (c) 2019 Amlogic, Inc. All rights reserved.
#
my $FILE;
$flist = "  ";
open($FILE, '<&STDIN');
while (<$FILE>) {
	chomp;
	my $line = $_;
	my $new = "";
	if($line =~/^A\s+(.+\.[cChH])/)
	{
		$new = $1;
	}
	elsif(/^A\s+(.+\.dts*)/i)
	{
		$new = $1;
	}
	if( -e $new)
	{
		$flist = $flist.$new."  ";
	}

}
close $FILE;

if($flist =~/^\s*$/)
{
	#print "\n LicenceCheck exit:No *.[CcHh] added.\n\n";
	exit 0;
}
else
{
	print "\n Check :$flist\n";
}

$match = "Licence_WARN: <";
$pl = "./scripts/amlogic/licence_check.pl";
$out = 0;
$result_0 = `$pl --nofix $flist`;
if($result_0 =~/$match/)
{
	$out =1;
	print $result_0;
	print "\n  Licence Check Error, please try to fix:\n  $pl $flist\n\n"
}
else
{
	print "\n  Licence Check OK\n\n"
}
exit $out;
