#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <string>
#include <iomanip>
using namespace cv;
using namespace std;
//=================================
class base
{
    protected:
    Mat world;    
    int base_size;   

    public:

        void set_feild();
        base(int size = 1000) : base_size(size) {}
        base(const base &b) {base_size = b.base_size;}
        ~base() {}
};
//===============================
void base :: set_feild()
{
    int size = 800;
    // cout<<"Enter base feild size : ";
    // cin>>size;
    base_size = size;
    world = Mat(size * 8/11 , size, CV_8UC3, Scalar(0, 0, 0));
    //===============================
    // declaring feild's point
    Point base_start(0,0);
    Point base_finish(base_size - 1 , base_size * 8/11 - 1);
    Point feild_start(base_size * 1/11 , base_size * 1/11);
    Point feild_finish(base_size * 10/11 , base_size * 7/11);
    Point mline_start(base_size / 2.0 , base_size * 1/11);
    Point mline_finish(base_size / 2.0 , base_size * 7/11);
    Point middle_circle(base_size / 2.0 , base_size * 4/11);
    Point middle_point(base_size / 2.0 , base_size * 4/11);
    Point r_penalty_start(base_size * 1/11 , base_size * 1.5/11);
    Point r_penalty_finish(base_size * 3/11 , base_size * 6.5/11);
    Point l_penalty_start(base_size * 10/11 , base_size * 6.5/11);
    Point l_penalty_finish(base_size * 8/11 , base_size * 1.5/11);
    Point r_goalarea_start(base_size * 1/11 , base_size * 2.5/11);
    Point r_goalarea_finish(base_size * 2/11 , base_size * 5.5/11);
    Point l_goalarea_start(base_size * 10/11 , base_size * 2.5/11);
    Point l_goalarea_finish(base_size * 9/11 , base_size * 5.5/11);
    Point r_goal_start(base_size * 0.4/11 , base_size * 2.7/11);
    Point r_goal_finish(base_size * 1/11 , base_size * 5.3/11);
    Point l_goal_start(base_size * 10/11 , base_size * 2.7/11);
    Point l_goal_finish(base_size * 10.6/11 , base_size * 5.3/11);
    Point mode_position(base_size * 4.5/11 , base_size * 0.5/11);
    // creat feild shapes
    rectangle(world, base_start, base_finish, Scalar(255, 255, 255), 2, 8, 0);
    rectangle(world, feild_start, feild_finish, Scalar(255, 255, 255), 2, 8, 0);
    line(world, mline_start, mline_finish, Scalar(255, 255, 255), 2, 8, 0);
    circle(world, middle_circle, (base_size * 1.5/22), Scalar(255, 255, 255), 2, 8, 0);
    circle(world, middle_point, (base_size * 0.02/22), Scalar(255, 255, 255), LINE_AA);
    rectangle(world, r_penalty_start, r_penalty_finish, Scalar(255, 255, 255), 2, 8, 0);
    rectangle(world, l_penalty_start, l_penalty_finish, Scalar(255, 255, 255), 2, 8, 0);
    rectangle(world, l_goal_start, l_goal_finish, Scalar(255, 255, 0), 2, 8, 0);
    rectangle(world, r_goal_start, r_goal_finish, Scalar(0, 255, 255), 2, 8, 0);
    rectangle(world, r_goalarea_start, r_goalarea_finish, Scalar(255, 255, 255), 2, 8, 0);
    rectangle(world, l_goalarea_start, l_goalarea_finish, Scalar(255, 255, 255), 2, 8, 0);

    putText(world, ".:: Soccer Feild ::.", mode_position, FONT_HERSHEY_DUPLEX, base_size * 0.015/22, Scalar(255, 255, 0));
}
//=================================
class robot : public base
{
    private:

        Mat Arrow;
        Point center,main,rigth,left;
        double robot_size;
        double center_x, center_y;
        double rigth_x, rigth_y;
        double left_x, left_y;
        double main_x, main_y;
        int robot_angel;
        int rotation_speed, move_speed;
        string pivot_x, pivot_y;
        Point left_corner, right_corner;
        int mouse_flag;
        int ps_x,ps_y;
        Point pointer;
        double tetha;

    public:

        void set_data();
        void robot_shape(string);
        double Radian(double);
        int robot_real_move();
        int robot_model_move();
        int check_boudary();
        void check_speed();
        void check_pivot();
        void moveSignal();
        static void static_mouse (int ,int ,int ,int ,void *);
        int mouse(int ,int ,int ,int);

};
//=================================
void robot :: set_data()
{
    robot_size = base_size * 0.3/11;
    center_x = base_size * 5/10;
    center_y = base_size * 4/11;
    ps_x = center_x;
    ps_y = center_y;
    robot_angel = 0;
    rotation_speed = 0;
    move_speed = 0;
}
//=================================
void robot :: robot_shape(string window)
{

    // creat a Arrow as a robot
    world.copyTo(Arrow);
    
    // declaring center point
    center = Point(center_x, center_y);

    // declaring main line of Arrow
    main_x = center_x + robot_size * cos(Radian(robot_angel));
    main_y = center_y + robot_size * sin(Radian(robot_angel));
    main = Point(main_x, main_y);

    // declaring right point of Arrow
    rigth_x = center_x + robot_size * cos(Radian(robot_angel - 140));
    rigth_y = center_y + robot_size * sin(Radian(robot_angel - 140));
    rigth = Point(rigth_x, rigth_y);

    // declaring left point of Arrow
    left_x = center_x + robot_size * cos(Radian(robot_angel + 140));
    left_y = center_y + robot_size * sin(Radian(robot_angel + 140));
    left = Point(left_x, left_y);

    // declaring info point
    Point x_position(base_size * 1/11, base_size * 7.3/11);
    Point y_position(base_size * 1/11, base_size * 7.6/11);
    Point x_point(base_size * 2.1/11, base_size * 7.3/11);
    Point y_point(base_size * 2.1/11, base_size * 7.6/11);

    check_pivot();

    Point move_speed_position(base_size * 3.3/11, base_size * 7.3/11);
    Point move_point(base_size * 5.7/11, base_size * 7.3/11);
    Point rotate_speed_position(base_size * 3.3/11, base_size * 7.6/11);
    Point rotate_point(base_size * 6.1 / 11, base_size * 7.6 / 11);
    string move_string = to_string(move_speed * 10);
    string rotate_string = to_string(rotation_speed);

    Point pixel_position(base_size * 7.5 / 11, base_size * 7.3/11);
    string position_x = to_string(ps_x);
    string position_y = to_string(ps_y);
    //==================================
    // rotation info
    Point angel_point(base_size * 7.5/ 11, base_size * 7.6/11);
    int set = -robot_angel + 180;
    if (set >= 360 || set <= -360)
    {
        robot_angel = 180;
    }
    if (set >= 180)
    {
        set -= 360;
    }
    if (set < -180)
    {
        set += 360;
    }
    string angel_string = to_string(set);

    //=========================

    // creat main lines of Arrow
    line(Arrow, main, rigth, Scalar(0, 0, 240), 2, 8, 0);
    line(Arrow, left, main, Scalar(0, 0, 240), 2, 8, 0);

    // creat sub lines
    line(Arrow, center, left, Scalar(0, 0, 240), 2, 8, 0);
    line(Arrow, rigth, center, Scalar(0, 0, 240), 2, 8, 0);

    // info section

    putText(Arrow, "Pivot X : " + pivot_x, x_position, FONT_HERSHEY_DUPLEX, base_size * 0.015 / 22, Scalar(0, 200, 0));
    putText(Arrow, "Pivot Y : " + pivot_y, y_position, FONT_HERSHEY_DUPLEX, base_size * 0.015 / 22, Scalar(0, 200, 0));


    //putText(Arrow, "Const Move Speed : 10 px/ms", move_speed_position, FONT_HERSHEY_DUPLEX, base_size * 0.013 / 22, Scalar(255, 255, 0));
    //putText(Arrow, "Const Rotation Speed : 5 deg/ms", rotate_speed_position, FONT_HERSHEY_DUPLEX, base_size * 0.013 / 22, Scalar(255, 255, 0));
    putText(Arrow, "Move Speed : "+move_string+" px/ms", move_speed_position, FONT_HERSHEY_DUPLEX , base_size * 0.015/22 , Scalar(255,255,0));
    putText(Arrow, "Rotation Speed : "+rotate_string+" deg/ms", rotate_speed_position, FONT_HERSHEY_DUPLEX , base_size * 0.015/22 , Scalar(255,255,0));

    putText(Arrow, "Pixel Position : (" + position_x + "," + position_y + ")", pixel_position, FONT_HERSHEY_DUPLEX, base_size * 0.015 / 22, Scalar(0, 0, 200));
    putText(Arrow, "Robot angel : " + angel_string + " deg", angel_point, FONT_HERSHEY_DUPLEX, base_size * 0.015 / 22, Scalar(0, 0, 200));

    // showing base picture
    //imshow("Model", Arrow);
    imshow("Real World", Arrow);

}
double robot :: Radian(double angel)
{
    angel *= 0.0174533;
    return angel;
}
//==================================
void robot :: static_mouse (int event, int x, int y,int flags ,void *data)
{
    robot* ptr = reinterpret_cast<robot*>(data);
    ptr->mouse(event, x, y ,flags);

}
int robot :: mouse(int event, int x, int y ,int flags) {
    if(check_boudary() == 0) {
        waitKey(1);
        if(mouse_flag == 0) {

            switch (event){

                case EVENT_LBUTTONDOWN : {
                    pointer.x = x;
                    pointer.y = y;
                    double distance = sqrt(pow(center_x-x, 2) + pow(center_y-y, 2));
                        if(distance < 20 ) {
                            robot_size += 10;
                            ps_x = x;
                            ps_y = y;
                            mouse_flag = 1;
                        }
                        else {
                            circle(world, pointer, (base_size * 0.02/22), Scalar(255, 255, 255), LINE_AA);
                            tetha = atan(center_y-y)/(center_x-x);
                            while(center_x <= pointer.x && center_y <= pointer.y){
                                //center_x += 1 * cos(Radian(tetha));
                                //center_y += 1 * sin(Radian(tetha));
                                center_x++;
                                center_y++;
                                
                                if(center_x <= pointer.x && !(center_y <= pointer.y))
                                    center_y++;
                                else if(center_y <= pointer.y && !(center_x <= pointer.x))
                                    center_x++;
                                else continue;
                               // robot_shape("Real World");
                                //usleep(1000);
                            }
                            //while(center_x >= pointer.x){
                                //center_x --;
                                //robot_shape("Real World");
                            //}
                        }
                        break;
                    }           
                }
            }

        else {

            switch(event) {

                case EVENT_MOUSEMOVE : {
                        center_x = x;
                        center_y = y;
                    break;
                }
                case EVENT_LBUTTONDBLCLK : {
                    robot_angel -= 10;
                    break;
                }
                case EVENT_RBUTTONDBLCLK : {
                    robot_angel += 10;
                    break;
                }

                case EVENT_MBUTTONDOWN : {
                    robot_size -= 10;
                    ps_x = x;
                    ps_y = y;
                    setMouseCallback("Real World", NULL, this);
                    moveSignal(); 
                    break;
                }
            }
        }
    }

    else {
        cout<<"\nMovement is out of feild !!\n";
    }

    robot_shape("Real World");
    return 0;  
}
//==================================
void robot :: moveSignal() {
    robot_shape("Real World");
    setMouseCallback("Real World", static_mouse,this); 
    mouse_flag = 0;
}
//=================================
int robot :: robot_real_move() {

    if(check_boudary() == 0) {
        robot_shape("Real World");
        switch(waitKey(1)) {

            case 97 :
                // change robot rotation
                rotation_speed--;
                break;
            
            case 100 :
                // change robot rotation
                rotation_speed++;
                break;
            
            case 119 :
                // change robot position
                move_speed++;
                //robot_shape();

                break;
            
            case 115 :
                // change robot position
                move_speed--;
                break;
            
            case 114 :
                // stop case
                move_speed = 0;
                rotation_speed = 0;
                robot_angel = robot_angel;
                break;
            
            case 98 :
                // reset case
                move_speed = 0;
                rotation_speed = 0;
                robot_angel = 0;
                center_x = base_size * 5/10;
                center_y = base_size * 4/11;
                break;
            
            case 113 :
                // To quit the window
                destroyAllWindows();
                return 0;
                break;
            
        }
    check_speed();
    
    }

    else {
        cout<<"\nMovement is out of feild !!\n";
    }
        
    robot_real_move();
    return 0;
}
//=================================
int robot :: robot_model_move() {
    
        robot_shape("Model");
        switch(waitKey(1)) {

            case 97 :{
                // change robot rotation
                rotation_speed--;
                break;
            }
            case 100 :{
                // change robot rotation
                rotation_speed++;
                break;
            }
            case 119 :{
                // change robot position
                move_speed++;
                //robot_shape();

                break;
            }
            case 115 :{
                // change robot position
                move_speed--;
                break;
            }
            case 114 :{
                // stop case
                move_speed = 0;
                rotation_speed = 0;
                robot_angel = robot_angel;
                break;
            }
            case 98 :{
                // reset case
                move_speed = 0;
                rotation_speed = 0;
                robot_angel = 0;
                center_x = base_size * 5/10;
                center_y = base_size * 4/11;
                break;
            }
            case 113 :{
                // To quit the window
                destroyAllWindows();
                return 0;
                break;
            }
        }
    check_speed();

    robot_model_move();
    return 0;
}
//====================================
int robot ::check_boudary()
{

    if (center_x <= base_size * 1 / 11) {
        center_x += base_size * 0.05 / 11;
        return 1;
    }

    else if (center_y <= base_size * 1 / 11) {
        center_y += base_size * 0.05 / 11;
        return 1;
    }

    else if (center_x >= base_size * 10 / 11) {
        center_x -= base_size * 0.05 / 11;
        return 1;
    }

    else if (center_y >= base_size * 7 / 11) {
        center_y -= base_size * 0.05 / 11;
        return 1;
    }

    return 0;
}
//=================================
void robot :: check_pivot()
{

    ostringstream set_x, set_y;

    if(center_x < base_size/2 && center_y < base_size*4/11){
            set_x<<setprecision(2)<<-(center_x * 11 / base_size - 5.5);
            set_y<<setprecision(2)<<center_y * 11 / base_size - 4;
        }
        else if(center_x > base_size/2 && center_y < base_size*4/11) {
            set_x<<setprecision(2)<<(center_x * 11 / base_size - 5.5);
            set_y<<setprecision(2)<<center_y * 11 / base_size - 4;
        }
        else if(center_x > base_size/2 && center_y > base_size*4/11) {
            set_x<<setprecision(2)<<(center_x * 11 / base_size - 5.5);
            set_y<<setprecision(2)<<-(center_y * 11 / base_size - 4);
        }
        else if(center_x < base_size/2 && center_y > base_size*4/11) {
            set_x<<setprecision(2)<<-(center_x * 11 / base_size - 5.5);
            set_y<<setprecision(2)<<-(center_y * 11 / base_size - 4);
        }
        else{
            set_x << 0;
            set_y << 0;
        }

    pivot_x = set_x.str();
    pivot_y = set_y.str();
}
//=================================
void robot ::check_speed()
{

    usleep(5000);
    if (move_speed <= 0)
    {
        center_x -= 1 * -move_speed * cos(Radian(robot_angel));
        center_y -= 1 * -move_speed * sin(Radian(robot_angel));    

    }
    else
    {
        center_x += 1 * move_speed * cos(Radian(robot_angel));
        center_y += 1 * move_speed * sin(Radian(robot_angel));
    }
    //=====================
    if (rotation_speed <= 0 && rotation_speed > -179)
    {
        robot_angel += 1 * rotation_speed;
    }
    else if (rotation_speed > 0 && rotation_speed <= 180)
    {
        robot_angel -= 1 * -rotation_speed;
    }
}
//=================================

int main()
{
    robot r1;

    r1.set_feild();
    r1.set_data();
    r1.moveSignal();
    r1.robot_real_move();
    //r1.robot_model_move();
    
    waitKey(0); 
    
    return 0;
}
