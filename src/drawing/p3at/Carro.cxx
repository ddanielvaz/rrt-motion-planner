#include "Carro.hh"

Carro::Carro(CarGeometry *car)
{
    //cout << "Criando instancia da classe Carro." << endl;
    retangulo[0] = cvPoint(SCALE_FACTOR * car->xlt, SCALE_FACTOR * car->ylt);
    retangulo[1] = cvPoint(SCALE_FACTOR * car->xrt, SCALE_FACTOR * car->yrt);
    retangulo[2] = cvPoint(SCALE_FACTOR * car->xrb, SCALE_FACTOR * car->yrb);
    retangulo[3] = cvPoint(SCALE_FACTOR * car->xlb, SCALE_FACTOR * car->ylb);
    fig[0] = retangulo;
}

void Carro::draw(IplImage* img, CvScalar color)
{
    int npts=4, ncurves = 1;
    cvPolyLine(img, fig, &npts, ncurves, 1, color);
}