package org.firstinspires.ftc.teamcode.SubSystems;

public class MotionProfile {

    private double max_a, max_v, dist, elap_t;


    public double Motion_Profile(double max_a, double max_v, double dist, double elap_t) {

        //Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.

        // Calculate the time it takes to accelerate to max velocity
        double a_dt = max_v / max_a;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double half_dist = dist/2;
        double a_dist = 0.5 * max_a * (Math.pow(a_dt, 2));

        if(a_dist > half_dist){
            a_dt = Math.sqrt(half_dist / (0.5 * max_a));
        }

        a_dist = 0.5 * max_a * (Math.pow(a_dt, 2));

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_v = max_a * a_dt;

        // we decelerate at the same rate as we accelerate
        double de_a_dt = a_dt;

        // calculate the time that we're at max velocity
        double cruise_dist = dist - 2 * a_dist;
        double cruise_dt = cruise_dist / max_v;
        double de_a_t = a_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = a_dt + cruise_dt + de_a_dt;

        if(elap_t > entire_dt)
            return dist;

        // if we're accelerating
        if(elap_t < a_dt)
            return 0.5 * max_a * (Math.pow(elap_t, 2));

            // if we're cruising
        else if (elap_t < de_a_t) {
            a_dist = 0.5 * max_a * (Math.pow(a_dt, 2));

            double cruise_current_dt = elap_t - a_dt;

            // use the kinematic equation for constant velocity
            return (a_dist + max_v * cruise_current_dt);
        }

        // if we're decelerating
        else {
            a_dist = 0.5 * max_a * (Math.pow(a_dt, 2));
            cruise_dist = max_v * cruise_dt;
            de_a_t = elap_t - de_a_t;

            // use the kinematic equations to calculate the instantaneous desired position
            return (a_dist + cruise_dist + max_v * de_a_t - 0.5 * max_a * (Math.pow(de_a_t, 2)));
        }

    }


}
