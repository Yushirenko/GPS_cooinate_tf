/* $Header: /ninebot_gps_xy.cpp                            March 2nd, 2021 10:33a Dylan YANG $ */
/***********************************************************************************************
 ***             C O N F I D E N T I A L  ---  I N S T A 3 6 0  R O B O T I C S              ***
 ***********************************************************************************************
 *                                                                                             *
 *                 Project Name : Following Robot                                              *
 *                                                                                             *
 *                    File Name : ninebot_gps_xy.cpp                                           *
 *                                                                                             *
 *                   Programmer : Dingyu Dylan YANG                                            *
 *                                                                                             *
 *                   Start Date : March 2nd, 2021                                              *
 *                                                                                             *
 *                  Last Update : March 2nd, 2021 [HKT]                                        *
 *                                                                                             *
 *---------------------------------------------------------------------------------------------*
 * Functions:                                                                                  *
 *     GPS_data_subscriber - Subscribe GPS data                                                * 
 *     GPS_XY_transformer  - Transform GPS data to XY                                          *
 *     XY_publisher        - Send XY point to ROS                                              *
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/***********************************************************************************************
 ***                                      INCLUDE                                            ***
 *---------------------------------------------------------------------------------------------*/
#include <math.h>                          /*                    Cpp headers                   */
#include <stdio.h>                         /*                                                  */
#include <iostream>                        /*--------------------------------------------------*/
//#include <ros/ros.h>                       /*                    ROS headers                   */
//#include <geometry_msgs/Pose.h>            /*                                                  */
//#include <geometry_msgs/Point.h>           /*--------------------------------------------------*/

/***********************************************************************************************
 ***                                       DEFINE                                            ***
 *---------------------------------------------------------------------------------------------*
 *        |     Name    |       Value      |                        Details                    *
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#define    PI_F         3.1415926f         /* The specific value of pi                         */
#define    R2D(v)       (v/PI_F*180.0f)    /* Converting radian to diameter                    */
#define    D2R(v)       (v/180.0f*PI_F)    /* Converting diameter to radian                    */
#define    Rad_lat      6378137.0f         /* Radius of the latitude circle                    */  
#define    Rad_lon      6356755.0f         /* Radius of the longitude circle                   */

/***********************************************************************************************
 ***                                       TYPEDEF                                           ***
 *---------------------------------------------------------------------------------------------*
 *   Data type   |    Name    |                             Details                            *
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
typedef struct {
    unsigned char inited;      /* Initalization parameters                                     */
    double        latitude;    /* Latitude of geography                                        */
    double        longitude;   /* Longitude of geography                                       */
    double        mue;         /* Radius of cuvature of meridian circle                        */
    double        lambada;     /* Radius of cuvature of parallel circle                        */
}geography_coordinate_tf;

/***********************************************************************************************
 ***                                      FUNCTIONS                                          ***
 *---------------------------------------------------------------------------------------------*/
int geography_coodinate_transform_init(geography_coordinate_tf *pref, double lon, double lat, double h){
    double e, e_2, f, sin_2_lat, R_lat_circle, R_lon_circle, den_prime_vertical, R_prime_vertical, omiga_2;
    
    if (lat > PI_F/2 || lat < -PI_F/2){
        return -1;
    }
    
    f = (Rad_lat - Rad_lon) / Rad_lat;
    e_2 = f * (2.0 - f);
    sin_2_lat = sin(lat);

    omiga_2 = 1 - e_2 * sin_2_lat;
    den_prime_vertical = sqrtf(omiga_2);
    R_prime_vertical = Rad_lat / den_prime_vertical;

    R_lat_circle = (R_prime_vertical + h) * cosf(lat);
    R_lon_circle = R_prime_vertical * (1 - e_2) / omiga_2 + h;

    pref->latitude  = lat;
    pref->longitude = lon;
    pref->mue       = R_lon_circle;
    pref->lambada   = R_lat_circle;
    pref->inited    = 1;
    
    return 0;
}

int geography_coodinate_transform_project(const geography_coordinate_tf *pref, double lon, double lat, float *x, float *y){
    if(!pref->inited){
        return -1;
    }

    *x = pref->mue * lon;
    *y = pref->lambada * lat;

    return 0;
}

int geography_coodinate_transform_reproject(const geography_coordinate_tf *pref, float x, float y, double *lon, double *lat){
    if(!pref->inited){
        return -1;
    }
    
    *lon = (double)x / pref->mue;
    *lat = (double)y / pref->lambada;

    return 0;
}

int geography_coodinate_transform_distance(const geography_coordinate_tf *pref, double lon, double lat, float *d_x, float *d_y){
    if(!pref->inited){
        return -1;

    }

    double d_lon, d_lat;
    d_lon = lon - pref->longitude;
    d_lat = lat - pref->latitude;

    *d_x = d_lon * pref->mue;
    *d_y = d_lat * pref->lambada;

    return 0;
}

/***********************************************************************************************
 ***                                         MAIN                                            ***
 *---------------------------------------------------------------------------------------------*/

geography_coordinate_tf geo_coor_ref = { 0 };

int main()
{
	/* 深圳 */
	double lat_s = 22.55329;
	double lon_s = 113.88308;

	/* 广州 */
	double lat_g = 23.15792;
	double lon_g = 113.27324;

	float x_g, y_g, x_s, y_s;

	geography_coodinate_transform_init(&geo_coor_ref, D2R(lon_s), D2R(lat_s), 10);
	printf("R-lat:%.3f, \r\nR-lon:%.3f\n", geo_coor_ref.lambada, geo_coor_ref.mue);

	geography_coodinate_transform_project(&geo_coor_ref, D2R(lon_g), D2R(lat_g), &x_g, &y_g);
	printf("g  x:%f,  y:%f\r\n",  x_g, y_g);

	geography_coodinate_transform_project(&geo_coor_ref, D2R(lon_s), D2R(lat_s), &x_s, &y_s);
	printf("g-->s  x:%f,  y:%f\r\n", x_g - x_s, y_g - y_s);

	float d1,d2;
	geography_coodinate_transform_distance(&geo_coor_ref, D2R(lon_g), D2R(lat_g), &d1, &d2);
	printf("g-->s  x:%f,  y:%f\r\n", d1, d2);
}
