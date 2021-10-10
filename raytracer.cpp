#include <iostream>
#include <math.h>

#include "raytracer.h"

double max(double a, double b)
{
    if (a > b)
    {
        return a;
    }
    return b;
}

Eigen::Vector3d trace(Eigen::Vector3d ray, Eigen::Vector3d color, double Ks, int recNum, Eigen::Vector3d bg, std::vector<Polygon> poly, 
    Eigen::Vector3d point, std::vector<Eigen::Vector3d> light)
{
    Eigen::Vector3d totalColor;

    double hit = 999, hitt = 0, beta, gamma, hitNum = -1;

    if (Ks > 0 && recNum != 5)
    {
        Eigen::Vector3d hitPoint;
        Eigen::Vector3d LightDir;

        for (unsigned int i = 0; i < poly.size(); i++)
        {
            hitt = raytri(ray, poly[i].getCoords()[0], poly[i].getCoords()[1], poly[i].getCoords()[2], point, &beta, &gamma);
            if (hitt > 0 && hitt < hit)
            {
                hit = hitt;
                hitNum = i;
            }
        }
        if (hitNum != -1)
        {
            Eigen::Vector3d vd;
            Eigen::Vector3d N;

            double red = 0;
            double green = 0;
            double blue = 0;
            double specular = 0;
            double lastSpec = 0;

            for (unsigned int l = 0; l < light.size(); l++)
            {
                N = (poly[hitNum].getCoords()[1] - poly[hitNum].getCoords()[0]).cross(poly[hitNum].getCoords()[2] - poly[hitNum].getCoords()[0]).normalized();
                hitPoint = (ray * hit + point * 1.01).normalized();
                LightDir << (light[l](0) - hitPoint(0)), (light[l](1) - hitPoint(1)), (light[l](2) - hitPoint(2));
                LightDir.normalize();

                double diffuse = 0;

                hit = 0;
                hitt = 0;

                for (unsigned int m = 0; m < poly.size(); m++)
                {
                    hitt = raytri(LightDir, poly[m].getCoords()[0], poly[m].getCoords()[1], poly[m].getCoords()[2], hitPoint, &beta, &gamma);
                    if (hitt > 0)
                    {
                        hit = hitt;
                    }
                }

                if (hit == 0)
                {
                    diffuse = max(0, N.dot(LightDir));
                    Eigen::Vector3d H((ray + LightDir).normalized());
                    lastSpec = specular;
                    specular = pow(max(0, N.dot(H)), poly[hitNum].getColor(5));
                    red += ((poly[hitNum].getColor(3) * poly[hitNum].getColor(0) * diffuse + poly[hitNum].getColor(4) * specular) * (1 / sqrt(light.size())));
                    green += ((poly[hitNum].getColor(3) * poly[hitNum].getColor(1) * diffuse + poly[hitNum].getColor(4) * specular) * (1 / sqrt(light.size())));
                    blue += ((poly[hitNum].getColor(3) * poly[hitNum].getColor(2) * diffuse + poly[hitNum].getColor(4) * specular) * (1 / sqrt(light.size())));
                }
            }
            Eigen::Vector3d tc(red, green, blue);
            totalColor = tc;

            if (specular > 0 || lastSpec > 0)
            {
                vd = (hitPoint - point).normalized();
                Eigen::Vector3d ReflectRay = -vd + 2 * (vd.dot(N)) * N;
                totalColor += Ks * trace(ReflectRay.normalized(), totalColor, poly[hitNum].getColor(4), recNum + 1, bg, poly, hitPoint, light);
            }
        }
        else
        {
            return bg;
        }
    }

    return totalColor;
}

void splitString(std::string toSplit, char delim, std::vector<std::string> *storeSplit)
{
        std::stringstream tmpData;
        std::string tmp;
        tmpData << toSplit;
        while (std::getline(tmpData, tmp, delim)) 
        {
            storeSplit->push_back(tmp);
        }
}

// Open the nff file and parse the data
int parseNff(std::string image, Eigen::Vector3d *background, std::vector<Eigen::Vector3d> *viewpoint, std::vector<Polygon> *polygon, std::vector<Eigen::Vector3d> *light)
{
    std::fstream nff_file;
    nff_file.open(image, std::ios::in);

    if (!nff_file.is_open())
    {
        std::cout << "File unable to be opened\n";
    }
    else
    {
        std::string data;

        // Get background info
        std::getline(nff_file, data);
        std::vector<std::string> backgroundData;
        splitString(data, ' ', &backgroundData);
        Eigen::Vector3d b(stof(backgroundData[1]), stof(backgroundData[2]), stof(backgroundData[3]));
        *background = b;

        // Get viewpoint info
        std::getline(nff_file, data);
        // next 6 getlines should correspond to from, at, up, angle, hither and resolution.
        for (int i = 0; i < 6; i++)
        {
            // Get info
            std::getline(nff_file, data);
            std::vector<std::string> viewpointData;
            splitString(data, ' ', &viewpointData);

            // first 3 have all three numbers, angle and hither only have 1 and res has 2
            if (i < 3) 
            {
                Eigen::Vector3d v(stod(viewpointData[1]), stod(viewpointData[2]), stod(viewpointData[3]));
                viewpoint->push_back(v);
            }
            else if (i < 5) 
            {
                Eigen::Vector3d v(stof(viewpointData[1]), 0, 0);
                viewpoint->push_back(v);
            }
            else
            {
                Eigen::Vector3d v(stof(viewpointData[1]), stof(viewpointData[2]), 0);
                viewpoint->push_back(v);
            }
        }

        std::vector<float> colorData;

        // Get polygons
        while (std::getline(nff_file, data))
        {
            // Get fill color info
            std::vector<std::string> fillData;
            splitString(data, ' ', &fillData);

            while (fillData[0] == LIGHT)
            {
                Eigen::Vector3d l(stof(fillData[1]), stof(fillData[2]), stof(fillData[3]));
                light->push_back(l);
                fillData.clear();
                std::getline(nff_file, data);
                splitString(data, ' ', &fillData);
            }

            Polygon p;
            
            // check if next data is polygon or color info
            if (fillData[0] == FILL_COLOR)
            {
                colorData.clear();
                for (unsigned int i = 1; i < fillData.size(); i++) 
                {
                    colorData.push_back(stof(fillData[i]));
                }  
                fillData.clear();
                std::getline(nff_file, data);
                splitString(data, ' ', &fillData);
            }
            p.setColor(colorData);

            // Set size of polygon
            if (fillData[0] == POLYGON || fillData[0] == POLY_PATCH)
            {
                p.setType(fillData[0]);
                int loopNum = stoi(fillData[1]);
                p.setSize(3);

                // Set coords
                std::vector<float> fcoordsData;
                std::vector<float> normData;
                for (int i = 0; i < loopNum; i++)
                {
                    std::getline(nff_file, data);
                    std::vector<std::string> coordsData;
                    splitString(data, ' ', &coordsData);
                    // Set coords to float

                    for (unsigned int j = 0; j < coordsData.size(); j++)
                    {
                        if (fillData[0] == POLYGON || j < (coordsData.size() / 2))
                        {
                            fcoordsData.push_back(stof(coordsData[j]));
                        }
                        else
                        {
                            normData.push_back(stof(coordsData[j]));
                        }
                    }
                    // Support for polygons > 3 sides
                    if (i >= 2)
                    {
                        if (i > 2)
                        {
                            Polygon po;
                            po.setColor(colorData);
                            po.setSize(3);
                            po.setCoords(fcoordsData);
                            po.setNorms(normData);
                            po.setType(fillData[0]);
                            polygon->push_back(po);
                        }
                        else
                        {
                            p.setNorms(normData);
                            p.setCoords(fcoordsData);
                            // add polygon to vector
                            polygon->push_back(p);
                        }

                        fcoordsData.clear();

                        fcoordsData.push_back(p.getCoords()[0][0]);
                        fcoordsData.push_back(p.getCoords()[0][1]);
                        fcoordsData.push_back(p.getCoords()[0][2]);
                        fcoordsData.push_back(p.getCoords()[2][0]);
                        fcoordsData.push_back(p.getCoords()[2][1]);
                        fcoordsData.push_back(p.getCoords()[2][2]);

                        if (fillData[0] == POLY_PATCH)
                        {
                            normData.clear();

                            normData.push_back(p.getNorms()[0][0]);
                            normData.push_back(p.getNorms()[0][1]);
                            normData.push_back(p.getNorms()[0][2]);
                            normData.push_back(p.getNorms()[2][0]);
                            normData.push_back(p.getNorms()[2][1]);
                            normData.push_back(p.getNorms()[2][2]);
                        }
                    }
                }
            }
            // spheres
            else if (fillData[0] == SPHERE)
            {
                p.setType(SPHERE);
                std::vector<float> fcoordsData;

                for (unsigned int i = 1; i < fillData.size(); i ++)
                {
                    fcoordsData.push_back(stof(fillData[i]));
                }

                p.setscoords(fcoordsData);
                polygon->push_back(p);
            }
        }
    }

    nff_file.close();
    return 0;
}

void camCoords(std::vector<Eigen::Vector3d> viewpoint, Camera *origin)
{
    Eigen::Vector3d w = (viewpoint[AT] - viewpoint[FROM]).normalized();
    Eigen::Vector3d u = (viewpoint[UP].cross(w)).normalized();
    Eigen::Vector3d v = (u.cross(w));

    origin->setW(w);
    origin->setU(u);
    origin->setV(v);
}

double raytri(Eigen::Vector3d rDir, Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3, Eigen::Vector3d origin, double *bet, double *gam)
{
    double a = v1(0) - v2(0);
    double b = v1(1) - v2(1);
    double c = v1(2) - v2(2);
    double d = v1(0) - v3(0);
    double e = v1(1) - v3(1);
    double f = v1(2) - v3(2);
    double g = rDir(0);
    double h = rDir(1);
    double ii = rDir(2);

    double jj = v1(0) - origin(0);
    double kk = v1(1) - origin(1);
    double l = v1(2) - origin(2);

    // M = a(ei - hf) + b(gf - di) + c(dh - eg)
    double M = (a * ((e * ii) - (h * f))) + (b * ((g * f) - (d * ii))) + (c * ((d * h) - (e * g)));

    double t = ((f * ((a * kk) - (jj * b))) + (e * ((jj * c) - (a * l))) + (d * ((b * l) - (kk * c)))) / M;
    if (t < 0 || t > 999) 
    {
        return 0;
    }

    double gamma = ((ii * ((a * kk) - (jj * b))) + (h * ((jj * c) - (a * l))) + (g * ((b * l) - (kk * c)))) / M;
    if (gamma < 0 || gamma > 1)
    {
        return 0;
    }

    double beta = ((jj * ((e * ii) - (h * f))) + (kk * ((g * f) - (d * ii))) + (l * ((d * h) - (e * g)))) / M;
    if (beta < 0 || beta > 1 - gamma)
    {
        return 0;
    }

    *bet = beta;
    *gam = gamma;

    return t;
}

double raySphere(Ray r, Eigen::Vector3d origin, Polygon p)
{
    double t1 = 0;
    double t2 = 0;
    Eigen::Vector3d c(p.getscoords()[0], p.getscoords()[1], p.getscoords()[2]);
    double R = p.getscoords()[3];

    double a = r.getDir().dot(r.getDir());
    double b = 2 * (r.getDir().dot(c - origin));
    double cc = (c - origin).dot(c - origin) - (R * R);
    double disc = (b * b) - (4 * a * cc);

    // sphere intersection
    if (disc >= 0)
    {
        t1 = ((-b + sqrt(disc)) / (2 * a));
        t2 = ((-b - sqrt(disc)) / (2 * a));
    }

    if (t1 <= t2)
    {
        return t1;
    }
    else
    {
        return t2;
    }
}

// get all pixel data and see if any intersect with a polygon
void pixels(int resx, int resy, int angle, double pixelWidth, std::vector<Ray> *rays, Camera coords, Eigen::Vector3d origin, 
            std::vector<Polygon> poly, std::vector<std::vector<HitRecord> > *hitRecord, Eigen::Vector3d bg, std::vector<Eigen::Vector3d> light)
{
    double aa, bb, cc;
    for (int i = 0; i < resy; i++)
    {   
        std::vector<HitRecord> hrv;
        for (int j = 0; j < resx; j++)
        {
            // Create ray
            Ray r;
            cc = -1;
            aa = (-tan((angle / 2) * M_PI / 180) + (pixelWidth / 2) + (i * pixelWidth));
            bb = ((tan((angle / 2) * M_PI / 180) / (resx / resy)) - (pixelWidth / 2) - (j * pixelWidth));
            
            r.setDir((aa * coords.getU()) + (bb * coords.getV()) + (cc * coords.getW()));
            r.setOrigin(origin);
            rays->push_back(r);

            HitRecord hr;
            hr.setT(999);

            Eigen::Vector3d c(bg[0], bg[1], bg[2]);
            hr.setColor(c);

            // Find if ray intersects with polygon
            for (unsigned int k = 0; k < poly.size(); k++)
            {
                double intersect;
                double beta;
                double gamma;
                if (poly[k].getType() == SPHERE)
                {
                    intersect = raySphere(r, origin, poly[k]);
                }
                else
                {
                    intersect = raytri(r.getDir(), poly[k].getCoords()[0], poly[k].getCoords()[1], poly[k].getCoords()[2], origin, &beta, &gamma);
                }

                // Only change color if the polygon of interest is closer than a different intersect
                if (intersect > 0 && intersect < hr.getT())
                {
                    // Eigen::Vector3d N = ((1 - beta - gamma) * poly[k].getNorms()[0]) + (beta * poly[k].getNorms()[1]) + (gamma * poly[k].getNorms()[2]);
                    Eigen::Vector3d N = (poly[k].getCoords()[1] - poly[k].getCoords()[0]).cross(poly[k].getCoords()[2] - poly[k].getCoords()[0]).normalized();
                    Eigen::Vector3d LightDir;
                    Eigen::Vector3d point;

                    double diffuse = 0;
                    double red = 0;
                    double green = 0;
                    double blue = 0;
                    double specular = 0;
                    double lastSpec = 0;

                    for (unsigned int l = 0; l < light.size(); l++)
                    {
                        point = -r.getDir() * intersect + r.getOrigin() * 1.01;
                        LightDir << (light[l](0) - point(0)), (light[l](1) - point(1)), (light[l](2) - point(2));
                        LightDir.normalize();

                        double hit = 0;
                        for (unsigned int m = 0; m < poly.size(); m++)
                        {
                            double hitt = raytri(-LightDir, poly[m].getCoords()[0], poly[m].getCoords()[1], poly[m].getCoords()[2], point, &beta, &gamma);
                            if (hitt > 0)
                            {
                                hit = hitt;
                            }
                        }

                        if (hit == 0)
                        {
                            diffuse = max(0, N.dot(LightDir));
                            Eigen::Vector3d H((r.getDir() + LightDir).normalized());
                            lastSpec = specular;
                            specular = pow(max(0, N.dot(H)), poly[k].getColor(5));
                            // specular = 0;
                            red += ((poly[k].getColor(3) * poly[k].getColor(0) * diffuse + poly[k].getColor(4) * specular) * (1 / sqrt(light.size())));
                            green += ((poly[k].getColor(3) * poly[k].getColor(1) * diffuse + poly[k].getColor(4) * specular) * (1 / sqrt(light.size())));
                            blue += ((poly[k].getColor(3) * poly[k].getColor(2) * diffuse + poly[k].getColor(4) * specular) * (1 / sqrt(light.size())));
                        }
                    }

                    Eigen::Vector3d co(red, green, blue);
                    if (specular > 0 || lastSpec > 0)
                    {
                        Eigen::Vector3d vd = (point - origin).normalized();
                        Eigen::Vector3d ReflectRay = -vd + 2 * (vd.dot(N)) * N;
                        co += poly[k].getColor(4) * trace(ReflectRay.normalized(), co, poly[k].getColor(4), 0, bg, poly, point, light);
                        // co += bg * poly[k].getColor(4);
                    }

                    co(0) > 1 ? co(0) = 1 : co(0);
                    co(1) > 1 ? co(1) = 1 : co(1);
                    co(2) > 1 ? co(2) = 1 : co(2);

                    hr.setColor(co);
                    hr.setT(intersect);
                }
            }
            hrv.push_back(hr);
        }
        hitRecord->push_back(hrv);
        
        // double percent = (i + 1) * 100;
        // std::cout << percent / resy << "%" << " done\n";
    }
}

int main(int argc, char *argv[])
{
    Eigen::Vector3d background;
    std::vector<Eigen::Vector3d> viewpoint;
    std::vector<Polygon> polygon;
    std::vector<Eigen::Vector3d> lights;
    Camera cCoords;
    std::vector<Ray> rays;
    std::vector<std::vector<HitRecord> > hitRecord;
    std::string renderImage = argv[1];
    std::cout << "Parsing nff file: " << renderImage << "\n";
    parseNff(renderImage, &background, &viewpoint, &polygon, &lights);
    std::cout << "Done parsing\nTotal polygons: " << polygon.size() << "\nTracing rays\n";

    const int height = viewpoint[RES](1, 0);
    const int width = viewpoint[RES](0, 0);

    // FIX TO USE HEIGHT AND WIDTH
    unsigned char pixel[height][width][3];

    double l = tan((viewpoint[ANGLE][0]/2) * M_PI / 180);
    double pixelWidth = (l * 2) / viewpoint[RES][0];

    // Compute cam coords
    camCoords(viewpoint, &cCoords);

    // Calculate pixel, ray and ray-polygon intersections and populate hitrecord data
    pixels(viewpoint[RES][0], viewpoint[RES][1], viewpoint[ANGLE][0], pixelWidth, &rays, cCoords, viewpoint[FROM], polygon, &hitRecord, background, lights);
    std::cout << "Done tracing\n";

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            pixel[i][j][0] = hitRecord[j][i].getColor()(0, 0) * 255;
            pixel[i][j][1] = hitRecord[j][i].getColor()(1, 0) * 255;
            pixel[i][j][2] = hitRecord[j][i].getColor()(2, 0) * 255;
        }
    }

    FILE *f = fopen("output.ppm", "wb");
    fprintf(f, "P6\n%d %d\n%d\n", width, height, 255);
    fwrite(pixel, 1, height*width*3, f);
    fclose(f);

    return 0;
}