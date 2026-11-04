#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>

geometry_msgs::Twist velocitat;
geometry_msgs::Pose2D current_pose;

int mode = 0; // 0: Espera | 1: Seguiment | 2: Gir 360° | 3: Gir 180°
double start_x = 0.0, start_y = 0.0;
double previous_yaw = 0.0;
double angle_acumulat = 0.0;
double dist_recorreguda = 0.0;
double previous_x = 0.0;
double previous_y = 0.0;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (mode != 1) return;

    //Distància superior a la màxima del làser
    float min_dist = 4.0;
    int index_intrus = -1;

    //Detecció d'intrús. Considerarà intrús l'objecte que estigui més a prop.

    for (int i = 338; i <= 359; i++) {
        if (msg->ranges[i] > 0.2 && msg->ranges[i] < min_dist) {
            min_dist = msg->ranges[i];
            index_intrus = i;
        }
    }
    for (int i = 0; i <= 22; i++) {
        if (msg->ranges[i] > 0.2 && msg->ranges[i] < min_dist) {
            min_dist = msg->ranges[i];
            index_intrus = i;
        }
    }

    //Si no detecta cap intrús es mou.

    if (index_intrus == -1) {
        ROS_INFO("No s'ha detectat cap intrús.");
        velocitat.linear.x = 0.0;
        velocitat.angular.z = 0.0;
        return;
    }

    ROS_INFO("Intrús detectat davant a %f metres", min_dist);

    //Manté sempre una distància mínima, retrocedint o avançant quan calgui.

    if (min_dist > 0.6) {
        velocitat.linear.x = 0.2;
    } else if (min_dist < 0.5) {
        velocitat.linear.x = -0.2;
    } else {
        velocitat.linear.x = 0.0;
    }

    //Per evitar que xoqui.

    for (int i = 172; i <= 182; i++) {
        if (msg->ranges[i] > 0.01 && msg->ranges[i] < 0.2 && min_dist < 0.5) {
            velocitat.linear.x = 0.0;
            ROS_INFO("Obstacle darrere detectat, aturat per evitar col·lisions.");
        }
    }

    //Centrar-se respecte a l'intrús, gira lleugerament.

    if (index_intrus >= 0 && index_intrus <= 22) {
        velocitat.angular.z = 0.2;
    } else if (index_intrus >= 338 && index_intrus <= 359) {
        velocitat.angular.z = -0.2;
    }
}

void intruderCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
        ROS_INFO("Mode actual: %d", mode);

        //Inicialitzem les variables perque quan es canvia de mode es reinicia (en voltes sobretot).
        //Previous_yaw em permet tenir un valor anterior i calcular el delta angle correctament.
        if (mode == 0) {
            mode = 1;
            start_x = current_pose.x;
            start_y = current_pose.y;
            ROS_INFO("Canvi a MODE 1: seguiment.");
        } else if (mode == 1) {
            mode = 2;
            previous_yaw = current_pose.theta;
            angle_acumulat = 0.0;
            ROS_INFO("Canvi a MODE 2: gir de 360°.");
        } else if (mode == 2) {
            ROS_INFO("Ignorat: Ja estic girant.");
        }
    }
}

void manualTurnCallback(const std_msgs::Bool::ConstPtr& msg)
{
    //Per fer la volta de 180 graus sota ordre. Torno a inicialitzar les variables. 
    //Em serveix per preparar el gir.
    if (msg->data) {
        mode = 3;
        previous_yaw = current_pose.theta;
        angle_acumulat = 0.0;
        velocitat.linear.x = 0.0;
        velocitat.angular.z = 0.0;
        ROS_INFO("Rebut senyal manual: canvi a MODE 3: gir de 180°.");
    }
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_pose.theta = yaw;

    if (mode == 1) {
        //Càlcul de la distància recorreguda (és la fórmula de la distància entre dos punts a l'espai)
        double delta = sqrt(pow(current_pose.x - previous_x, 2) + pow(current_pose.y - previous_y, 2));
        dist_recorreguda += delta;

        ROS_INFO("Distància recorreguda: %.2f m", dist_recorreguda);

        previous_x = current_pose.x;
        previous_y = current_pose.y;

        //Si he recorregut més de set metres de distància, entro a mode 2.
        if (dist_recorreguda >= 7.0) {
            mode = 2;
            previous_yaw = yaw;
            angle_acumulat = 0.0;
            dist_recorreguda = 0.0;
            ROS_INFO("Distància superior a 7m. Canvi a MODE 2.");
        }
    }

    //Si estem en mode 2 o mode tres, controlem el gir.
    else if (mode == 2 || mode == 3) {
        double delta_angle = yaw - previous_yaw;

        //Correcció de signes per evitar errors.

        if (delta_angle > M_PI) delta_angle -= 2 * M_PI;
        if (delta_angle < -M_PI) delta_angle += 2 * M_PI;

        angle_acumulat += fabs(delta_angle);
        previous_yaw = yaw;

        ROS_INFO("Angle girat acumulat: %.2f radians", angle_acumulat);

        velocitat.linear.x = 0.0;
        velocitat.angular.z = 0.7;

        //Si el mode es dos, angle limit ha de valdre dos pi abans de parar, si no (mode 3), ha de valdre pi.

        double limit_angle = (mode == 2) ? 2 * M_PI : M_PI;

        //Si l'angle acumulat ja ha arribat, paro el gir.

        if (angle_acumulat >= limit_angle) {
            mode = 1;
            start_x = current_pose.x;
            start_y = current_pose.y;
            velocitat.angular.z = 0.0;
            ROS_INFO("Volta completada. Canvi a MODE 1.");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "seguiment_intrus");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("scan", 10, scanCallback);
    ros::Subscriber sub_odom = nh.subscribe("odom", 1000, odomCallback);
    ros::Subscriber sub_intrus = nh.subscribe("intruder_command", 10, intruderCallback);
    ros::Subscriber sub_manual = nh.subscribe("manual_turn", 10, manualTurnCallback);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Rate loop_rate(2);

    //En iniciar el programa.

    previous_x = current_pose.x;
    previous_y = current_pose.y;

    while (ros::ok())
    {
        pub.publish(velocitat);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}