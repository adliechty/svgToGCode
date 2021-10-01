# svgToGCode
This takes an SVG file (main current use case is generated from boxes.py) and will export G code meant for a CNC router, auto adding tabs at  logical locations (adjacent to other tabs) so that the tabs are not cut by the adjacent path.  It also has a finishing step at the end where the router plunge cuts all the tabs so you don't have to manually do it with a chissel.
