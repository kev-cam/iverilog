#!/usr/bin/env perl
#
# Regression script for VHDL output using NVC simulator.
# Based on vhdl_reg.pl (GHDL version).
#

use lib './perl-lib';

use RegressionList;
use Diff;
use Reporting;
use Environment;

my $NVC = $ENV{NVC} || "/usr/local/src/nvc/build/bin/nvc";
my $NVC_LIBDIR = $ENV{NVC_LIBDIR} || "/usr/local/src/nvc/build/lib";

# Support running from the build tree: if ../build-lib exists, use it
my $IVL_BUILD_LIB = $ENV{IVL_BUILD_LIB} || "";
if (!$IVL_BUILD_LIB && -d "../build-lib") {
    $IVL_BUILD_LIB = "../build-lib";
}

#
#  Main script
#
&open_report_file('vhdl_nvc_regression_report.txt');
my ($suffix, $strict, $with_valg, $force_sv) = &get_args;
my $ver = &get_ivl_version($suffix);
&print_rpt("Running VHDL/NVC tests for Icarus Verilog version: $ver.\n");
&print_rpt("-" x 70 . "\n");
if ($#ARGV != -1) {
    my $regress_fn = &get_regress_fn;
    &read_regression_list($regress_fn, $ver, $force_sv, "");
} else {
    &read_regression_list("vhdl_regress.list", $ver, $force_sv, "");
}
&execute_regression($suffix, $with_valg);
&close_report_file;


sub execute_regression {
    my $sfx = shift(@_);
    my $with_valg = shift(@_);
    my ($tname, $total, $passed, $failed, $expected_fail, $not_impl,
        $len, $cmd, $diff_file, $outfile, $unit);

    $total = 0;
    $passed = 0;
    $failed = 0;
    $expected_fail = 0;
    $not_impl = 0;
    $len = 0;

    mkdir 'vhdl' unless (-d 'vhdl');

    foreach $tname (@testlist) {
        $len = length($tname) if (length($tname) > $len);
    }

    foreach $tname (@testlist) {
        next if ($tname eq "");

        $total++;
        &print_rpt(sprintf("%${len}s: ", $tname));
        if ($diff{$tname} ne "" and -e $diff{$tname}) {
            unlink $diff{$tname} or
                die "Error: unable to remove old diff file $diff{$tname}.\n";
        }
        if (-e "log/$tname.log") {
            unlink "log/$tname.log" or
                die "Error: unable to remove old log file log/$tname.log.\n";
        }

        if ($testtype{$tname} eq "NI") {
            &print_rpt("Not Implemented.\n");
            $not_impl++;
            next;
        }

        $outfile = "vhdl/$tname.vhd";

        # Run iverilog to generate VHDL
        my $bflag = $IVL_BUILD_LIB ? "-B$IVL_BUILD_LIB " : "";
        $cmd = "iverilog$sfx ${bflag}-t vhdl -o $outfile $args{$tname}";
        $cmd .= " -s $testmod{$tname}" if ($testmod{$tname} ne "");
        $cmd .= " ./$srcpath{$tname}/$tname.v > log/$tname.log 2>&1";
        if (system("$cmd")) {
            if ($testtype{$tname} eq "CE") {
                if ($? >> 8 & 128) {
                    &print_rpt("==> Failed - CE (core dump).\n");
                    $failed++;
                } else {
                    &print_rpt("Passed - CE.\n");
                    $passed++;
                }
                next;
            }
            $cmd = "grep -q -i -E '(no vhdl translation|cannot be translated)' log/$tname.log";
            if (system($cmd) == 0) {
                &print_rpt("==> Failed - No VHDL translation.\n");
                $not_impl++;
                next;
            }
            else {
                &print_rpt("==> Failed - running iverilog.\n");
                $failed++;
                next;
            }
        }

        # Compile with NVC (analyze)
        $cmd = "(cd vhdl ; $NVC --std=2040 -L $NVC_LIBDIR -a $tname.vhd) >> log/$tname.log 2>&1";
        if (system("$cmd")) {
            if ($testtype{$tname} eq "CE") {
                if ($? >> 8 & 128) {
                    &print_rpt("==> Failed - CE (core dump).\n");
                    $failed++;
                } else {
                    &print_rpt("Passed - CE.\n");
                    $passed++;
                }
                next;
            }
            &print_rpt("==> Failed - NVC analyze.\n");
            $failed++;
            next;
        }

        # CO test type: compilation only
        if ($testtype{$tname} eq "CO") {
            &print_rpt("Passed - CO.\n");
            $passed++;
            next;
        }

        # Find the primary (top-level) entity from the VHDL file.
        # Entities are emitted leaves-first, so the top entity is last.
        ($unit) = `grep -oP '^entity\\s+\\K\\w+' $outfile | tail -1`;
        chomp $unit if $unit;
        unless ($unit) {
            &print_rpt("==> Failed -- cannot determine primary VHDL unit.\n");
            $failed++;
            next;
        }

        # Elaborate
        $cmd = "(cd vhdl ; $NVC --std=2040 -L $NVC_LIBDIR -e $unit) >> log/$tname.log 2>&1";
        if (system($cmd)) {
            &print_rpt("==> Failed - NVC elaborate.\n");
            $failed++;
            next;
        }

        # Run simulation
        $cmd = "(cd vhdl ; $NVC --std=2040 -L $NVC_LIBDIR -r $unit --stop-delta=10000) >> log/$tname.log 2>&1";
        if (system($cmd)) {
            if ($testtype{$tname} eq "RE") {
                &print_rpt("Passed - RE.\n");
                $passed++;
                next;
            }
            else {
                $cmd = "grep -q 'SIMULATION FINISHED' log/$tname.log";
                if (system($cmd)) {
                    &print_rpt("==> Failed - NVC simulate.\n");
                    $failed++;
                    next;
                }
            }
        }

        if ($diff{$tname} ne "") {
            $diff_file = $diff{$tname}
        } else {
            $diff_file = "log/$tname.log";
        }
        if (diff($gold{$tname}, $diff_file, $offset{$tname})) {
            if ($testtype{$tname} eq "EF") {
                &print_rpt("Passed - expected fail.\n");
                $expected_fail++;
                next;
            }
            &print_rpt("==> Failed - output does not match gold file.\n");
            $failed++;
            next;
        }

        &print_rpt("Passed.\n");
        $passed++;

    } continue {
        if ($tname ne "") {
            system("rm -rf ./vhdl/work") and
                die "Error: failed to remove temporary files.\n";
        }
    }

    &print_rpt("=" x 70 . "\n");
    &print_rpt("Test results:\n  Total=$total, Passed=$passed, Failed=$failed,".
               " Not Implemented=$not_impl, Expected Fail=$expected_fail\n");
}
