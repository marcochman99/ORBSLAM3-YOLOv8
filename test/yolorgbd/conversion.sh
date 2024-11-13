#awk '{$2 = -$2; $3 = -$3;} 1' data.txt > output.txt
#above mutiply -1 to column 2 AND  3 of txt file, make sure to remove existing KeyFrmaeTrajectory generate by slam itself, write protected
