#ifndef PLACE_CONFIG_H
#define PLACE_CONFIG_H

struct Joint {
    double j1 = 0.0;
    double j2 = 0.0;
    double j3 = 0.0;
    double j4 = 0.0;
    double j5 = 0.0;
    double j6 = 0.0;
};

struct PlaceConfig
{
    struct
    {
        int layerX = 0;
        int layerY = 0;
        double obj_length = 0.0;
        double obj_width = 0.0;
        double obj_height = 0.0;
    } ArrayConfig;

    Joint PassJoint;
    Joint PlaceJoint;
    Joint WaitJoint;
};

#endif // PLACE_CONFIG_H
