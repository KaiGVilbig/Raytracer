#include <vector>
#include <Eigen/Dense>

class Polygon
{
    public:
        void setSize(int);
        int getSize();

        void setCoords(std::vector<float>);
        std::vector<Eigen::Vector3d> getCoords();

        void setNorms(std::vector<float>);
        std::vector<Eigen::Vector3d> getNorms();

        void setColor(std::vector<float>);
        float getColor(int);

        void setType(std::string);
        std::string getType();

        void setscoords(std::vector<float>);
        std::vector<float> getscoords();

    private:
        int polySize;
        std::vector<Eigen::Vector3d> coords;
        std::vector<Eigen::Vector3d> norms;
        std::vector<float> scoords;
        std::vector<float> color;
        std::string type;
};

class Ray
{
    public:
        Eigen::Vector3d getDir();
        void setDir(Eigen::Vector3d);

        Eigen::Vector3d getOrigin();
        void setOrigin(Eigen::Vector3d);

    private:
        Eigen::Vector3d dir;
        Eigen::Vector3d origin;
};

class HitRecord
{
    public:
        void setT(double);
        double getT();

        void setColor(Eigen::Vector3d);
        Eigen::Vector3d getColor();

    private:
        double t;
        Eigen::Vector3d color;
};

class Camera
{
    public:
        Eigen::Vector3d getW();
        void setW(Eigen::Vector3d);

        Eigen::Vector3d getU();
        void setU(Eigen::Vector3d);

        Eigen::Vector3d getV();
        void setV(Eigen::Vector3d);

    private:
        Eigen::Vector3d w;
        Eigen::Vector3d u;
        Eigen::Vector3d v;
};