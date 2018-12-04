#ifndef CONSTANTPARAMS_H
#define CONSTANTPARAMS_H

// Constantes usadas para los circulos
#define C_THRES_CANNY       100     // Threshold usado en Canny
#define C_FACTOR_CANNY      3       // Factor de Canny

#define C_MIN_ASPECT_RATIO  0.5     // Relacion 2:1 entre el largo y ancho del bounding box
#define C_MIN_RECTAN        0.7     // Rectangularidad (circulo -> 0,7853975)
#define C_MIN_AREA          0.2


// Constantes usadas para los anillos
#define R_PAR_MIN_ASPECT_RATIO      0.5     // Factor aspect ratio minimo del anillo padre
#define R_CHD_MIN_ASPECT_RATIO      0.4
#define R_CUR_MIN_ASPECT_RATIO      0.55
#define R_PAR_MIN_RECTAN    0.7     // Factor de rectangularidad
#define R_CHD_MIN_RECTAN    0.4
#define R_CUR_MIN_RECTAN    0.75
#define R_CUR_MIN_AREA      0.1

#endif // CONSTANTPARAMS_H
