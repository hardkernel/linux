#!/usr/bin/perl -W
# SPDX-License-Identifier: (GPL-2.0+ OR MIT)
#
# Copyright (c) 2019 Amlogic, Inc. All rights reserved.
#
use File::Basename;
use File::Find;

$sc_dir = File::Spec->rel2abs(dirname( "$0") ) ;
$sc_dir =~ s/\/scripts\/amlogic//;
my $top = "$sc_dir";

my $nofix = 0;
my $failno = 0;
my $shname = $0;
#@ARGV=("../../include/linux/amlogic","../../drivers/amlogic" ) ;
my @path;
for(@ARGV)
{
	my $dir	=$_;
	if(/^\//)
	{
	}
	elsif(/--nofix/)
	{
		$nofix = 1;
		next;
	}
	else
	{
		$dir = File::Spec->rel2abs($dir);
		#print "\n Real2abs Dir: --$dir-- \n";
	}
	push(@path,$dir);
}


my $licence_start="// SPDX-License-Identifier: (GPL-2.0+ OR MIT)\n";
my $licence_end=
"/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 */\n\n";

#print $licence;

sub licence_process
{
	my ($file_name) = @_;
	my $d = dirname($file_name);
	my $f = basename($file_name);
	#print "\n Abs <$d>,  f_ $f";
	#print "\n Top: <$top>  ";

	$licence_0=$licence_start.$licence_end;
	my $count = 0;
	my $text_0="";
	my $text_all=$licence_0;
	open(my $f_in, '<', $file_name) or die "Can't Open $file_name: For Read \n";
	my ($left,$right, $lineno,$space) = (0, 0, 0,0);
	while ($line = <$f_in>)
	{
		$text_0 .= $line;
		#Empty Line or Line marked by //
		if(($space==0) &&(($line =~/^\s*$/)||
		(($line =~/^\s*\/\//)&&($line !~ /\*\//))))
		{
			#print "\n Line $lineno is empty.";
		}
		elsif(($space==0) &&($line =~ /^\s*\/\*/))											#Match /*
		{
			$left ++;
			#print "\n L Matched: $lineno  $line, $left  ";								#Match that /* and */ in the same line
			if($line =~ /\*\//)
			{
				$right ++;
				#print "\n L Matched: $lineno  $line, $left  ";
			}
		}
		elsif(($space==0) &&($line =~ /\*\//)&& ($line !~ /\/\*/) )													#Match */
		{
			$right ++;
			#print "\n R Matched: $lineno  $line, $right  ";
			if($left == $right)
			{
				$space = 1;
			}
		}
		elsif($left==$right)	#Content Lines
		{
			if(($line =~/^\s*$/)&& ($count==0))
			{

			}
			else
			{
				#print $line;
				$space = 1;
				$count +=1;
				$text_all .=$line;
			}
		}
		$lineno++;
	}
	close($f_in);

	if($text_0 ne $text_all)
	{
		$failno ++;
		if($nofix)
		{
			print "\n  Licence_WARN: <";
			print File::Spec->abs2rel($file_name, $top).">\n";;
		}
		else
		{
			print "\n  Licence_FIXED: <";
			print File::Spec->abs2rel($file_name, $top).">\n";;
			open(my $f_out, '>', $file_name)
			or die "Can't Open $file_name\n";
			print $f_out $text_all;
			close $f_out;
		}
	}
	$text_all='';
}


my ($c_cnt, $h_cnt) = (0, 0);
sub process
{
    my $file = $File::Find::name;
    if (-f $file)
     {
		if(($file =~ /.*\.[Cc]$/i) || ($file =~ /.*\.dtsi$/i) || ($file =~ /.*\.dts$/i))
		{
			$c_cnt++;
			$licence_start="// SPDX-License-Identifier: (GPL-2.0+ OR MIT)\n";
			licence_process($file);
		}
		if(($file =~ /.*\.[hH]$/i))
		{
			$c_cnt++;
			$licence_start="/* SPDX-License-Identifier: (GPL-2.0+ OR MIT) */\n";
			licence_process($file);
		}
	}
}

for(@path)
{
	#print "\n Fine $_ \n";
	find(\&process, $_);
}
