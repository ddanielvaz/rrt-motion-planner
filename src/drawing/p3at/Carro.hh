#ifndef __CARRO_HH__
#define __CARRO_HH__

#include <cv.h>

#include "Geometry.hh"
#include "DrawingUtils.hh"

class Carro
{
    public:
        Carro(CarGeometry*);
        void draw(IplImage*, CvScalar);
    private:
        CvPoint retangulo[4];
        CvPoint *fig[1];
};

#endif
