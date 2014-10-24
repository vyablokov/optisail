set term x11
set multiplot
set parametric
set xrange [-1000:1000]
set yrange [-1000:4000]
set size ratio 2.5
set title "Optimal course"
set arrow from 536.8,3659.57 to 250,3250
plot [0:2*pi] 10*sin(t)+(0),10*cos(t)+(2500) lt 1 notitle
plot 'waypoints.txt' w l lt 3 notitle
pause mouse
