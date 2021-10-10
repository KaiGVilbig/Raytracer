#include "objects.h"

void Polygon::setSize(int size)
{
    polySize = size;
}

int Polygon::getSize() 
{
    return polySize;
}

void Polygon::setCoords(std::vector<float> c)
{
    for (int i = 0; i < polySize * 3; i+=3)
    {
        Eigen::Vector3d a(c[i], c[i + 1], c[i + 2]);
        coords.push_back(a);
    }
}

std::vector<Eigen::Vector3d> Polygon::getCoords()
{
    return coords;
}

void Polygon::setNorms(std::vector<float> n)
{
    for (unsigned int i = 0; i < n.size(); i+=3)
    {
        Eigen::Vector3d a(n[i], n[i + 1], n[i + 2]);
        norms.push_back(a);
    }
}

std::vector<Eigen::Vector3d> Polygon::getNorms()
{
    return norms;
}

void Polygon::setColor(std::vector<float> c)
{
    for (unsigned int i = 0; i < c.size(); i++)
    {
        color.push_back(c[i]);
    }
}

float Polygon::getColor(int loc)
{
    return color[loc];
}

void Polygon::setType(std::string t)
{
    type = t;
}

std::string Polygon::getType()
{
    return type;
}

void Polygon::setscoords(std::vector<float> c)
{
    for (unsigned int i = 0; i < c.size(); i++)
    {
        scoords.push_back(c[i]);
    }
}

std::vector<float> Polygon::getscoords()
{
    return scoords;
}

Eigen::Vector3d Ray::getDir()
{
    return dir;
}

void Ray::setDir(Eigen::Vector3d d)
{
    dir = d;
}

Eigen::Vector3d Ray::getOrigin()
{
    return origin;
}

void Ray::setOrigin(Eigen::Vector3d o)
{
    origin = o;
}

Eigen::Vector3d Camera::getW()
{
    return w;
}

void Camera::setW(Eigen::Vector3d set)
{
    w = set;
}

Eigen::Vector3d Camera::getU()
{
    return u;
}

void Camera::setU(Eigen::Vector3d set)
{
    u = set;
}

Eigen::Vector3d Camera::getV()
{
    return v;
}

void Camera::setV(Eigen::Vector3d set)
{
    v = set;
}

double HitRecord::getT()
{
    return t;
}

void HitRecord::setT(double T)
{
    t = T;
}

Eigen::Vector3d HitRecord::getColor()
{
    return color;
}

void HitRecord::setColor(Eigen::Vector3d c)
{
    color = c;
}