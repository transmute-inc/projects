set terminal png size 1200,600
set output "k.png"
set title " Proton Current"
set margins screen 0.1, screen 0.8, screen 0.1, screen 0.94
set multiplot

set xlabel " Time sec"
set xtics
set auto x

set ylabel "H2 Pressure - Torr" tc "web-green" 
set ytics nomirror tc "web-green"
set autoscale y 

set y2label "Proton Current - microamps" tc "web-blue"
set y2tics nomirror offset -1,0 tc "web-blue"
set autoscale y2

set key left top
set grid x y2 
plot './I.dat' using 1:2 with linespoints title 'pressure' axes x1y1 lw 1 lc 'web-green', \
     './I.dat' using 1:3 with linespoints title 'current'  axes x1y2 lw 1 lc 'web-blue'

unset title
unset xlabel
unset ylabel
unset y2label
unset tics
unset grid

set key right top
set yrange[0:100]
set y2range[0:100]
plot './I.dat' using 1:4 with linespoints title 'Grid'    axes x1y1 lw 1 lc 'red', \
     './I.dat' using 1:5 with linespoints title 'Cathode' axes x1y2 lw 1 lc 'dark-violet'

set key off
  set rmargin at screen 0.9
  set border 8
  set y2label 'Voltage'  offset -1,0 tc "red"
  set y2tics nomirror offset 0,0 tc "red"
set yrange[0:100]
  plot sqrt(x*.001) lc "grey"

unset multiplot
unset output
