#include "Geometry.hh"

CarGeometry::CarGeometry(double width, double height, double length)
{
    cout << "Criando instancia da classe CarGeometry." << endl;
    w = width;
    h = height;
    l = length;
}
/**
 * A partir de um ponto de rotação (x,y) e de um ângulo de rotação theta,
 * gera os vertices de um retangulo de largura w, altura h.
 * @param x coordenada x, do ponto em torno do qual o retangulo será rotacionado.
 * @param y coordenada y, do ponto em torno do qual o retangulo será rotacionado.
 * @param theta ângulo de rotação.
*/
void CarGeometry::SetVerticesPosition(double x, double y, double theta)
{
    double rot_mat[4], xc, yc, aux_x, aux_y;
    //Clockwise rotation
    rot_mat[3] = rot_mat[0] = cos(theta);
    rot_mat[1] = -sin(theta);
    rot_mat[2] = sin(theta);

    xc=x + (w+l)/2.0;
    yc=y - h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xrt = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    yrt = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;

    xc=x - (w-l)/2.0;
    yc=y - h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xlt = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    ylt = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;

    xc=x - (w-l)/2.0;
    yc=y + h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xlb = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    ylb = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;

    xc=x + (w+l)/2.0;
    yc=y + h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xrb = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    yrb = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;
}

double CarGeometry::get_between_axes_length()
{
    return l;
}

double CarGeometry::get_body_width()
{
    return w;
}

double CarGeometry::get_body_height()
{
    return h;
}

CarGeometry::~CarGeometry()
{
    cout << "Destruindo instancia da classe CarGeometry" << endl;
}