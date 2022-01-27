Obtención de la nube de puntos mediante el uso de la estimación de la distancia con imagenes.

Dos principales aproximaciones:

Estimación por deep learning: un modelo de red neuronal llamado MiDaS (MiDaS v2.1 Large)
 obtiene un mapa de profundidad a partir de una imagen rgb.

Estimación por visión estereo, usando dos camaras se aproxima la distancia por la disparidad.

Para la generación de la nube de puntos se itera por cada imagen de una grabación y se añaden
los puntos tridimensionales teniendo en cuenta la posición de la cámara.