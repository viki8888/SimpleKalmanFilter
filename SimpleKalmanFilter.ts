/**
 * 简单的卡尔曼滤波.
 */
//% color=190 weight=100 icon="\uf0b0" block="卡尔曼滤波" Advanced =true
namespace SimpleKalmanFilter {
    
    
    export class kalman{
        private Q_angle:number;
        private Q_bias:number;
        private R_measure:number ;
        private p:number[][];
        private angle:number;
        private bias:number;
        private rate:number;

        constructor(engine:string) { 
            this.Q_angle = 0.001;
            this.Q_bias = 0.003;
            this.R_measure = 0.03;
            this.p=[[0,0],[0,0]];
            this.angle = 0; 
            this.bias = 0; 
            
        }

        getAngle(newAngle: number, newRate: number,dt:number): void{
            /* Step 1 */
            this.rate = newRate - this.bias;
            this.angle += dt * this.rate;

            /* Step 2 */
            this.P[0][0] += dt * (dt*this.P[1][1] - this.P[0][1] - this.P[1][0] + this.Q_angle);
            this.P[0][1] -= dt * this.P[1][1];
            this.P[1][0] -= this.dt * this.P[1][1];
            this.P[1][1] += this.Q_bias * dt;

            /* Step 4 */
            let S:number = this.P[0][0] + this.R_measure; // Estimate error
            
            /* Step 5 */
            let K:number[];
            K[0] = this.P[0][0] / S;
            K[1] = this.P[1][0] / S;

             /* Step 3 */
            let y:number = newAngle - this.angle; // Angle difference
            
            /* Step 6 */
            this.angle += K[0] * y;
            this.bias += K[1] * y;
            
            /* Step 7 */
            let P00_temp:number = P[0][0];
            let P01_temp:number = P[0][1];
            this.P[0][0] -= K[0] * P00_temp;
            this.P[0][1] -= K[0] * P01_temp;
            this.P[1][0] -= K[1] * P00_temp;
            this.P[1][1] -= K[1] * P01_temp;

            return angle;
        }

        setAngle(angle:number) { this.angle = angle; }; // Used to set angle, this should be set as the starting angle
        getRate() { return this.rate; }; // Return the unbiased rate

        /* These are used to tune the Kalman filter */
        setQangle(Q_angle:number) { this.Q_angle = Q_angle; };
        setQbias( Q_bias:number) { this.Q_bias = Q_bias; };
        setRmeasure(R_measure:number) { this.R_measure = R_measure; };
        getQangle() { return this.Q_angle; };
        getQbias() { return this.Q_bias; };
        getRmeasure() { return this.setRmeasure(0)R_measure; };
    
    } 
    

    export function Test(): void {
       console.log("123456") 
    }
   
}

