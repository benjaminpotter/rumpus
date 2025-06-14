set terminal pngcairo
set output "acc.png"

set title "Hough Transform Accumulator"
set xlabel "Solar Azimuth Angle [deg]"
set ylabel "Votes"
set xrange [-90:90]

plot 'acc.dat' with lines title ""
