import geopy.distance


edificio_1 = (40.543619,-4.011944)
edificio_2 = (40.543639,-4.011961)

estatua_1 = (40.543826,-4.01184)
estatua_2 = (40.543826,-4.011841)

coche_1 = (40.543829,-4.012041)
coche_2 = (40.543834,-4.012026)

d1 = geopy.distance.geodesic(edificio_1, edificio_2).m
print(d1)

d2 = geopy.distance.geodesic(estatua_1, estatua_2).m
print(d2)

d3 = geopy.distance.geodesic(coche_1, coche_2).m
print(d3)

