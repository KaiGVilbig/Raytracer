#include <string>
#include <fstream>
#include <vector>
#include <Eigen/Dense>

#include "objects.h"

std::string LIGHT = "l";
std::string FILL_COLOR = "f";
std::string POLYGON = "p";
std::string SPHERE = "s";
std::string POLY_PATCH = "pp";

int FROM = 0;
int AT = 1; 
int UP = 2;
int ANGLE = 3;
int HITHER = 4;
int RES = 5;

void splitString(std::string, char, std::vector<std::string>);
int parseNff(std::string, Eigen::Vector3d, std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>);
void pixels(int, int, int, double, std::vector<Ray>, Camera, Eigen::Vector3d, std::vector<Polygon>, std::vector<HitRecord>, Eigen::Vector3d);
void camCoords(std::vector<Eigen::Vector3d>, Camera);
double raytri(Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, double*, double*);