
set terminal pngcairo
set output "assets/img/aop.png"

set title "Angle of Polarization"

unset tics
set cbtics
set cbrange [-90:90]
set xrange [0:2448]
set yrange [0:2048]
set xlabel "Roll"
set ylabel "Pitch"

set view map
splot 'assets/dat/aop.dat' matrix with image title ''

