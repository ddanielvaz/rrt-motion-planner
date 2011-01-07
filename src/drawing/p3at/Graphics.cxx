#include "Graphics.hh"
#include "Carro.hh"
#include "Constants.hh"
#include "Obstacles.hh"

CvScalar colors[] = {CV_RGB(255,0,0), CV_RGB(0,255,0), CV_RGB(0,0,255),
                     CV_RGB(255,255,0), CV_RGB(0.596*255,0.984*255,0.596*255), CV_RGB(0,0,0),
                     CV_RGB(255,255,255) };

Graphics::Graphics(CarGeometry *car_geom, char *obs_file, char *title)
{
    cout << "Criando instancia da classe Graphics" << endl;
    // Criar instancia da classe Obstacle como atributo desta classe
    // Substituir 7.2 e 5.3 por leitura do arquivo de obstacles
    // Criar na classe obstacles dois atributos, Width e Height para armazenar
    // a largura e altura total do mapa.
    map_enviroment = new Obstacles(obs_file);
    CvSize s = cvSize(map_enviroment->w * SCALE_FACTOR + 1, map_enviroment->h * SCALE_FACTOR + 1);
    CvScalar white = colors[c_white];
    img1 = cvCreateImage(s, 8, 3);
    cvNamedWindow(title, CV_WINDOW_AUTOSIZE);
    cvRectangle(img1, cvPoint(0,0), cvPoint(map_enviroment->w * SCALE_FACTOR + 1, map_enviroment->h * SCALE_FACTOR + 1), white, -1, 8, 0);
    veh_geom = car_geom;
}

Graphics::~Graphics()
{
    cout << "Destruindo instancia da classe Graphics" << endl;
    cvReleaseImage(&img1);
    cvDestroyAllWindows();
}

void Graphics::draw_initial_and_goal(double *initial, double *goal)
{
    CvScalar green = colors[c_green], red = colors[c_red];
    double x, y, r=3.0;
    x = SCALE_FACTOR * initial[0];
    y = SCALE_FACTOR * initial[1];
    draw_trail(initial[0], initial[1], initial[2], c_black);
    cvCircle(img1, cvPoint(x,y), r, red, -1, 8, 0);
    x = SCALE_FACTOR * goal[0];
    y = SCALE_FACTOR * goal[1];
    draw_trail(goal[0], goal[1], goal[2], c_black);
    cvCircle(img1, cvPoint(x,y), r, green, -1, 8, 0);
}

void Graphics::plot_trail_states(char *filename, int color)
{
    char temp[100], *ps, *nxt;
    int i;
    float x, y, theta;
    ifstream fp(filename);
    if(!fp.is_open())
    {
        cout << "Arquivo " << filename << " nao encontrado." << endl;
        return;
    }
    cout << "Lendo pontos do arquivo. " << endl;
    //Lendo ponto inicial
    fp.getline(temp, 100);
    x = strtod(temp, &ps);
    nxt = ps;

    y = strtod(nxt, &ps);
    nxt = ps;

    theta = strtod(nxt, &ps);
    nxt = ps;
    draw_trail(x, y, theta, color);
    i = 1;
    //lendo ponto Target
    while(fp.getline(temp, 100))
    {
        x = strtod(temp, &ps);
        nxt = ps;

        y = strtod(nxt, &ps);
        nxt = ps;

        theta = strtod(nxt, &ps);
        nxt = ps;
        draw_trail(x, y, theta, color);
        i++;
    }
    cout << i << " pontos lidos." << endl;
    fp.close();
}

void Graphics::draw_trail(double x, double y, double theta, int color_id)
{
    //cvCircle(img1, cvPoint(x * SCALE_FACTOR,y * SCALE_FACTOR), 1.0, colors[c_red], -1, 8, 0);
    veh_geom->SetVerticesPosition(x, y, theta);
    Carro car(veh_geom);
    CvScalar color = colors[color_id];
    car.draw(img1, color);
}

void Graphics::plot_line_states(char *filename, int color)
{
    char temp[100], *ps, *nxt;
    int i;
    double x, y, theta_initial, x_old, y_old, theta;
    ifstream fp(filename);
    if(!fp.is_open())
    {
        cout << "Arquivo " << filename << " nao encontrado." << endl;
        return;
    }
    cout << "Lendo pontos do arquivo. " << endl;
    //Lendo ponto inicial
    fp.getline(temp, 100);
    x = x_old = strtod(temp, &ps);
    nxt = ps;

    y = y_old = strtod(nxt, &ps);
    nxt = ps;
    
    theta = theta_initial = strtod(nxt, &ps);
    nxt = ps;
    
    draw_trail(x_old, y_old, theta_initial, color);

    i = 1;
    //lendo ponto Target
    while(fp.getline(temp, 100))
    {
        x = strtod(temp, &ps);
        nxt = ps;

        y = strtod(nxt, &ps);
        nxt = ps;
        
        theta = strtod(nxt, &ps);
        nxt = ps;

        draw_line(x_old, y_old, x, y, color);
        x_old = x;
        y_old = y;
        i++;
    }
    draw_trail(x, y, theta, color);
    cout << i << " pontos lidos." << endl;
    fp.close();
}

void Graphics::plot_tree(char *filename, int color, RobotModel *r)
{
    char temp[100], *ps, *nxt;
    int i;
    double x[r->n_states], xf[r->n_states], u[2], integration_time;
    ifstream fp(filename);
    if(!fp.is_open())
    {
        cout << "Arquivo " << filename << " nao encontrado." << endl;
        return;
    }
    cout << "Lendo pontos do arquivo. " << endl;
    //Lendo ponto inicial
    fp.getline(temp, 100);
    x[STATE_X] = strtod(temp, &ps);
    nxt = ps;

    x[STATE_Y] = strtod(nxt, &ps);
    nxt = ps;

    x[STATE_THETA] = strtod(nxt, &ps);
    nxt = ps;
    
    x[STATE_V] = strtod(nxt, &ps);
    nxt = ps;

    x[STATE_W] = strtod(nxt, &ps);
    nxt = ps;

    u[0] = strtod(nxt, &ps);
    nxt = ps;

    u[1] = strtod(nxt, &ps);
    nxt = ps;

    integration_time = strtod(nxt, &ps);
    nxt = ps;
for(double t=0.0; t<=integration_time; t+=DELTA_T)
        {
            r->EstimateNewState(x, u, xf);
            draw_line(x[STATE_X], x[STATE_Y], xf[STATE_X], xf[STATE_Y], color);
            memcpy(x, xf, sizeof(double)*r->n_states);
        }
    //draw_trail(x_old, y_old, theta_initial, color);

    //i = 1;
    //lendo ponto Target
    while(fp.getline(temp, 100))
    {
        x[STATE_X] = strtod(temp, &ps);
        nxt = ps;

        x[STATE_Y] = strtod(nxt, &ps);
        nxt = ps;

        x[STATE_THETA] = strtod(nxt, &ps);
        nxt = ps;

        x[STATE_V] = strtod(nxt, &ps);
        nxt = ps;

        x[STATE_W] = strtod(nxt, &ps);
        nxt = ps;

        u[0] = strtod(nxt, &ps);
        nxt = ps;

        u[1] = strtod(nxt, &ps);
        nxt = ps;

        integration_time = strtod(nxt, &ps);
        nxt = ps;
        
        
        for(double t=0.0; t<=integration_time; t+=DELTA_T)
        {
            r->EstimateNewState(x, u, xf);
            draw_line(x[STATE_X], x[STATE_Y], xf[STATE_X], xf[STATE_Y], color);
            memcpy(x, xf, sizeof(double)*r->n_states);
        }
        i++;
    }
//     draw_trail(x, y, theta, color);
    cout << i << " pontos lidos." << endl;
    fp.close();
}

void Graphics::draw_line(double x_old, double y_old, double x, double y, int color_id)
{
    CvScalar color = colors[color_id];
    cvLine(img1, cvPoint(x_old* SCALE_FACTOR, y_old* SCALE_FACTOR), cvPoint(x* SCALE_FACTOR,y* SCALE_FACTOR), color);
}

void Graphics::plot_obstacles(void)
{
    map_enviroment->draw(img1);
}

void Graphics::show(char *title)
{
    cvShowImage(title, img1);
    cvSaveImage("trajetoria.png",img1);
    cvWaitKey(0);
}