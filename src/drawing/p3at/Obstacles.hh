#ifndef __OBSTACLES_HH__
#define __OBSTACLES_HH__

#include <cv.h>
#include <vector>

using namespace std;

class Obstacles
{
    public:
        Obstacles(char *);
        void draw(IplImage *);
        vector<CvPoint3D64f> triangs;
        double w, h;
};

#endif
