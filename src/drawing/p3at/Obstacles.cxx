#include "Obstacles.hh"
#include "DrawingUtils.hh"

#include <iostream>
#include <fstream>
using namespace std;

Obstacles::Obstacles(char *filename)
{
    cout << "Criando instancia da classe Obstacles" << endl;
    ifstream obs_fp(filename);
    if(!obs_fp.is_open())
    {
        cerr << "Arquivo: " << filename << " nao encontrado. Podem ocorrer erros na visualizacao." << endl;
        return;
    }
    char temp[100], *ps, *nxt;
    double v[9];
    int i;
    // Pegando informação sobre largura e altura total do ambiente
    obs_fp.getline(temp, 100);
    w = strtod(temp, &ps);
    h = strtod(ps, NULL);
    cout << w << " " << h << endl;
    while(obs_fp.getline(temp, 100))
    {
        v[0] = strtod(temp, &ps);
        nxt = ps;
        for (i=1; i<9; i++)
        {
            v[i] = strtod(nxt, &ps);
            nxt = ps;
        }
        for(i=0;i<9;i=i+3)
            triangs.push_back(cvPoint3D64f(v[i], v[i+1], v[i+2]));
    }
    obs_fp.close();
}

void Obstacles::draw(IplImage *img)
{
    CvPoint triang[3];
    CvPoint *fig[1];
    int npts=3, ncurves=1;
    CvScalar red = CV_RGB(255,0,0);
    CvScalar black = CV_RGB(0,0,0);
    for (vector<CvPoint3D64f>::iterator i = triangs.begin(); i != triangs.end(); ){
        triang[0] = cvPoint(SCALE_FACTOR * i->x, SCALE_FACTOR * i->y);
        ++i;
        triang[1] = cvPoint(SCALE_FACTOR * i->x, SCALE_FACTOR * i->y);
        ++i;
        triang[2] = cvPoint(SCALE_FACTOR * i->x, SCALE_FACTOR * i->y);
        ++i;
        fig[0] = triang;
        //Solid
        cvFillPoly(img, fig, &npts, ncurves, black);
        //Wireframe
        //cvPolyLine(img, fig, &npts, ncurves, 1, red);
    }
}