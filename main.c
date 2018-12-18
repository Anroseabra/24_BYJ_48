/*******************************************************
 *
 * main.c
 *
 *  Created on: 16/11/2018
 *      Author: Andreia Seabra e André Teixeira
 * 
 * main.c v2
 * 
 * Movimento dos dois motores nas interrupçoes
mak * Para fazer movimentar os motores fornecer-lhes:
 *   - motorx.step --- numero de steps
 *   - motorx.t_between_step --- tempo entre cada step csec
 *   - motorx.dir --- indica a direçao do morotor
 * main.c v2
 *  
 * Funções DegreetoRad , vice-versa
 *         Distancia entre dois pontos
 *         DirectKinematics
 * 
 * main.c v10.1
 *  -Reta (check)
 ******************************************************/
#include <avr/interrupt.h> /*Interrupts library */
#include <avr/io.h> /* Registers */
#include <util/delay.h> /* Delays library */

#include "printf_tools.h" 
#include "serial_printf.h" 
#include <math.h>

#define DEBUG

/*1st Motor coils*/
#define ORANGE1 PC0
#define YELLOW1 PC1
#define PINK1 PC2
#define BLUE1 PC3

/*2nd Motor coils*/
#define ORANGE2 PD7
#define YELLOW2 PD6
#define PINK2 PD5
#define BLUE2 PD4

#define TCONT 100

/*Testing...*/
#define NSTEP1 500
#define NSTEP2 500

#define ERROR 50

/*Motor variables*/
#define CLOCKWISE 1
#define COUNTERCLOCKWISE -1
#define NCOIL 8
/*Using half step mode, this motor has 360/5.625*64 = 4096 steps*/
#define DEGREE_PER_STEP 0.08789063 

/*Arm variables*/
#define ARM_SIZE 20
#define FOREARM_SIZE 20

/*Size of small straight*/
#define D_SCALE 0.1
#define UP 1
#define DOWN 0

/* Fim de curso*/

#define FM1 PD2 //INT 0 
#define FM2 PD3 //INT 1


#define TPREDEFI 10
#define TINICIAL 1000

#define TRUE 1
#define FALSE 0
#define T1BOTTOM 65536-2500 /*10ms*/

typedef struct{
    volatile uint16_t move; /*TRUE OR FALSE*/
    volatile uint16_t t_until_nmove; /*centiseconds*/
    volatile uint16_t t_between_step; /*centiseconds*/
    volatile uint16_t step;
    volatile int dir;
    volatile uint16_t coilindex;
}StepMotor;

typedef struct{
    float x;
    float y;
    double theta1;
    double theta2;
}Point;

StepMotor motor1 = {0,100,0,0,0,0};
StepMotor motor2 = {0,100,0,0,0,0};
Point Origin = {0, 0};
Point MyPos = {0,0}; //Origin

volatile uint16_t coil_m1[NCOIL]={(1<<BLUE1),               
                        (1<<BLUE1)|(1<<PINK1),
                        (1<<PINK1),
                        (1<<PINK1)|(1<<YELLOW1),
                        (1<<YELLOW1),
                        (1<<YELLOW1|1<<ORANGE1),
                        (1<<ORANGE1),
                        (1<<ORANGE1|1<<BLUE1)};
volatile uint16_t coil_m2[NCOIL]={(1<<BLUE2),
                        (1<<BLUE2|1<<PINK2),
                        (1<<PINK2),
                        (1<<PINK2|1<<YELLOW2),
                        (1<<YELLOW2),
                        (1<<YELLOW2|1<<ORANGE2),
                        (1<<ORANGE2),
                        (1<<ORANGE2|1<<BLUE2)};
/*kUSUME*/
volatile uint16_t counttimer1=0,counttimer2=0;

int p(float value){
    return (int)(value*1000);
}

void tc1_init(void){
    /*Iinicialização do timer 1*/

    TCCR1B = 0; //Stop timer 1 
    TIFR1 = (7<<TOV1)|(1<<ICF1); //Clear all pending interrupts
    TCCR1A = 0; /* Normal mode */
    TCNT1 = T1BOTTOM; /* Load botom value */
    TIMSK1 = (1<<TOIE1); /* Enable interrupts by overflow */
    TCCR1B = 3; /* timer prescaler = 100b */
    sei();
}

ISR(TIMER1_OVF_vect){ /* No interrupt do timer1 são executados os movimentos do motor*/
    
    TCNT1 = T1BOTTOM;
    
    motor1.t_until_nmove--; 
    motor2.t_until_nmove--;
    //printf("%d -- %d \n\r", motor1.t_until_nmove, motor2.t_until_nmove);
    /*Move motor1*/
    if(motor1.step>0 && motor1.move){

        PORTC=coil_m1[motor1.coilindex]; /*Movement*/

        if(motor1.dir == CLOCKWISE){ /*Update index*/
            if(motor1.coilindex == (NCOIL-1))
                motor1.coilindex=0; //starts again
            else
                motor1.coilindex++;
        }

        else{/*COUNTERCLOCKWISE movement*/ //printf("COUNTERCLOCKWISE\n\r");
            if(0==motor1.coilindex) 
                motor1.coilindex=NCOIL-1; /*Final Point*/
            else
                motor1.coilindex--;
        }
        motor1.step--;
        //printf("--Motor1 %d-- ", motor1.step);
        motor1.move=FALSE; 
    }
    
    if(motor2.step>0 && motor2.move){
        /*Move motor2*/
        PORTD=coil_m2[motor2.coilindex]|(1<<FM1)|(1<<FM2); /*Movement*/

        if(motor2.dir == CLOCKWISE){ /*Update index*/
            if((NCOIL-1)==motor2.coilindex)
                motor2.coilindex=0;
            else
               motor2.coilindex++;
        }

        else{ /*COUNTERCLOCK Movement*/
            if(0==motor2.coilindex)
                motor2.coilindex=NCOIL-1;
            else 
                motor2.coilindex--;
        }
        motor2.step--;
        //printf("--MOTOR2 %d -- \n\r", motor2.step);
        motor2.move=FALSE;   
    }
    /*Verifica se o motor deve mover na proxima interrupçao*/
    if(0==motor1.t_until_nmove){
        motor1.t_until_nmove=motor1.t_between_step;
        motor1.move=TRUE;
    }
    if(0==motor2.t_until_nmove){
        motor2.t_until_nmove=motor2.t_between_step;
        motor2.move=TRUE;
    }

}                 

void start_timer1(int count1){
    counttimer1 = count1;
}

int get_timer1(){
    if(0 == counttimer1){
        return 1;
    }
    return 0;
}

void hw_init(){

    DDRC = DDRC|(1<<ORANGE1)|(1<<PINK1)|(1<<YELLOW1)|(1<<BLUE1); 
    DDRD = (~(1<<FM1))|(~(1<<FM2));
    DDRD = DDRD|(1<<ORANGE2)|(1<<PINK2)|(1<<YELLOW2)|(1<<BLUE2); 
    PORTD = (1<<FM1)|(1<<FM2);  
    PORTC = 0;  

    EICRA = (2<<ISC00)|(2<<ISC10);
    EIMSK = (1<<INT0)|(1<<INT1);
    sei();
}

ISR(INT0_vect){
    motor1.step = 0 ;
}

ISR(INT1_vect){
    motor2.step = 0;
}
float dist(Point p, Point l){
    return sqrt(pow(l.x-p.x,2)+pow(l.y-p.y,2));
}

float RadtoDegree(float angle){
    return angle*180/M_PI;
}

float DegreetoRad(float angle){
    return angle*M_PI/180;
}

float LawOfCos(float a, float b, float c){
    //printf("lawofcos %d %d %d\n\r",(int)a, (int)b, (int)c);
    return acos( ( pow(a,2) + pow(b,2) - pow(c,2) ) / (2 * a *b) );
}

void EquationOfaStraightLine(Point MyPos, Point TargetPos, float *m, float *b){
    /*
    * y = mx + b
    */

    printf("Equation of a straight line\n\r");
    //printf("MyPos.x %d , MyPos.y %d , TargetPos.x %d , TargetPos.y %d\n\r ", (int)(1000*MyPos.x), (int)(1000*MyPos.y), (int)(1000*TargetPos.x), (int)(1000*TargetPos.y));
    if(MyPos.x == TargetPos.x){
        *m = 0;
        *b = MyPos.x;
    }
    else{
        *m = (TargetPos.y - MyPos.y) / (TargetPos.x-MyPos.x);
        *b = MyPos.y - (*m) * MyPos.x;
    }

}

int abs(int number){
   
    if(number<0)
        return -number;
    else
        return number;
}

int LCM (int a, int b){
    /*
    * It is known that LCM = a * b / gdc(a,b)
    * Where LCM is least common multiple and
    * GDC is greatest common divisor
    */
    if(0 == a){
       return b;
    }
    else if( 0 == b){
        return a;
    }
    else{
        int gdc, lcm, i;

        for(i=1; i <= a && i<=b; i++){
            if( (0 == a%i) && (0 == b%i))
                gdc = i;
        }

        lcm = a * b / gdc ;
        return lcm;
    }
    return -1;
}

void ChooseDir(int *dir, float angle_ini, float angle_fin){

    //printf("  Angle init %d Angle final %d", (int)(angle_ini*1000), (int)(angle_fin*1000));
    if(angle_fin >= angle_ini){
        *dir = CLOCKWISE;
    //    printf(" CLOCKWISE \n\r");
    }
    else{
        *dir = COUNTERCLOCKWISE;
    //    printf(" COUNTERCLOCKWISE \n\r");
    }
}

void DirectKinematics(double theta1, double theta2, float *x, float *y){
    /*Angles most come in degree*/

    double theta1_rad = DegreetoRad(theta1);
    double theta2_rad = DegreetoRad(theta2);
    *x = FOREARM_SIZE * cos(theta1_rad) + ARM_SIZE * cos(theta1_rad + theta2_rad);
    *y = FOREARM_SIZE * sin(theta1_rad) + ARM_SIZE * sin(theta1_rad + theta2_rad);
    
}

void InverseKinematics(Point robot, Point target, double *theta1, double *theta2){

    //printf("%d tx %d ty %d rx %d ry", (int)target.x, (int)target.y, (int)robot.x, (int)robot.y);
    float length = dist(robot, target);
    //printf("---%d distancia---", (int)length);
    if(length > (ARM_SIZE + FOREARM_SIZE)){
        printf("Too far\n\r");
    }
    else{
        float alpha = LawOfCos(length, FOREARM_SIZE, ARM_SIZE);
        //printf("%d alpha\n\r", (int)(alpha*1000));
        float beta = LawOfCos(ARM_SIZE, FOREARM_SIZE, length);
        //printf("%d beta\n\r", (int)(beta*1000));
        float gama = atan2 (target.y-robot.y,target.x-robot.x);
        //printf("%d gama\n\r", (int)(gama*1000));

        *theta1 = RadtoDegree(gama - alpha);
        *theta2 = RadtoDegree(M_PI - beta);
    }
}

void CalcTime_between_step(uint16_t m1_step, uint16_t m2_step, volatile uint16_t *t1, volatile uint16_t *t2){

    int t_total = LCM( m1_step, m2_step);
    
    /* Quando o numero de steps e nulo igualou-se o tempo a 1,
       mas na verdade poderia-se igual a qualquer outro valor,
       apenas e necessária esta atribuição pois nao e possivel
       dividir um numero por zero, tornando assim o valor de
       t_between_step previsel para todos os casos
    */
    (0 == m1_step) ? (*t1 = 1) : (*t1 = t_total / m1_step);
    (0 == m2_step) ? (*t2 = 1) : (*t2 = t_total / m2_step);

}
Point CalcSubTarget(int flag, float m, float b, int dir){
    
    /* dir - UP MyPos.x or y < Target.x or y
           - DOWN MyPos.x or y > Target.x or y*/
    Point SubTarget;

    if(UP == dir){
        if(0==flag){
            SubTarget.x = MyPos.x + D_SCALE;
            SubTarget.y = m * SubTarget.x + b;
        }
        else{
            //printf("%d---- m real\n\r", (int)(m*100000000));

            if(!m){
                SubTarget.y = MyPos.y + D_SCALE;
                SubTarget.x = MyPos.x;            
            }
            else{
                SubTarget.y = MyPos.y + D_SCALE;
                SubTarget.x = 1/m * SubTarget.y - b/m;
            }
        }
    }
    else{ 
        if(0==flag){
            SubTarget.x = MyPos.x - D_SCALE;
            SubTarget.y = m * SubTarget.x + b;
        }
        else{
            //printf("%d---- m real\n\r", (int)(m*100000000));

            if(!m){
                SubTarget.y = MyPos.y - D_SCALE;
                SubTarget.x = MyPos.x;            
            }
            else{
                SubTarget.y = MyPos.y - D_SCALE;
                SubTarget.x = 1/m * SubTarget.y - b/m;
            }
        }
    }

    InverseKinematics(Origin, SubTarget, &SubTarget.theta1, &SubTarget.theta2);

    return SubTarget;

}

void Calc_move(Point SubTarget ){
    /*
    * Preparado para utizar x ou y para controlar o comprimento das mini-retas:
    *   - flag = 0 -> controla em x;
    *   - flag = 1 -> controla em y;
    * 
    * Calcula o Target para a semi-reta (SubTarget);
    * Decide a direção de cada motor;
    * Calcula o numero de steps
    * Através do nº de steps obtem o t_between steps para cada motor
    * Utiliza o nºde steps efetuados para calcular a posição real
    * Atualiza a posiçãp para a real após o movimento
    * 
    */
    double theta1,theta2;
    
    StepMotor fake_m1 = {};
    StepMotor fake_m2 = {};

    ChooseDir(&fake_m1.dir, MyPos.theta1, SubTarget.theta1);
    ChooseDir(&fake_m2.dir, MyPos.theta2, SubTarget.theta2);

    /*Calculate the number of steps*/
    theta1 = MyPos.theta1 - SubTarget.theta1;
    theta2 = MyPos.theta2 - SubTarget.theta2;

    fake_m1.step = abs((int) (theta1 / DEGREE_PER_STEP+0.5));
    fake_m2.step = abs((int) (theta2 / DEGREE_PER_STEP+0.5));

    CalcTime_between_step(fake_m1.step, fake_m2.step, &fake_m1.t_between_step, &fake_m2.t_between_step);

    /*Updates MyPos*/

    MyPos.theta1 = fake_m1.step * DEGREE_PER_STEP * fake_m1.dir + MyPos.theta1;
    MyPos.theta2 = fake_m2.step * DEGREE_PER_STEP * fake_m2.dir + MyPos.theta2;

    DirectKinematics(MyPos.theta1, MyPos.theta2, &MyPos.x, &MyPos.y);

    /*Atualizaçao os motores*/

    /*Certerficar-me que não fazem nada enquanto os atualizo*/
    motor1.t_until_nmove = 10000;
    motor1.move = FALSE;

    motor2.t_until_nmove = 10000;
    motor2.move = FALSE;

    /* Atribui os valores pretendidos*/
    motor1.dir = fake_m1.dir;
    motor1.t_between_step = fake_m1.t_between_step;
    motor1.step = fake_m1.step;

    motor2.dir = fake_m2.dir;
    motor2.t_between_step = fake_m2.t_between_step;
    motor2.step = fake_m2.step;
    
    /*Inicia o movimento*/

    motor1.t_until_nmove = 1;
    motor2.t_until_nmove = 1;

    //#ifdef DEBUG
        printf(" -- SubTarget.theta1: %d SubTarget.theta2 %d SubTarget.x %d SubTarget.y %d-- \n\r" , (int)(SubTarget.theta1*1000), (int)(1000*SubTarget.theta2), (int)(1000*SubTarget.x),  (int)(1000*SubTarget.y));
        printf(" --- dir1 %d dir2 %d---- ", fake_m1.dir, fake_m2.dir );
        printf(" -- Angulos a mover %d %d --", (int)(theta1*1000), (int)(theta2*1000));
        printf(" -- Step1 %d  Step2 %d --", fake_m1.step, fake_m2.step);
        printf(" -- fake_m1.t_between_step %d -- fake_m2.t_between_step %d -- \n\r", fake_m1.t_between_step, fake_m2.t_between_step);
        printf(" -- MyPos.theta1: %d MyPos.theta2 %d MyPos.x %d MyPos.y %d-- \n\r" , (int)(MyPos.theta1*1000), (int)(1000*MyPos.theta2), (int)(1000*MyPos.x),  (int)(1000*MyPos.y));
        printf(" -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- \n\r");
    //#endif
}
int Make_move(Point TargetPos){
    
    /*
    * Calcula a reta que liga os pontos
    * Decide o eixo que controla a variação
    * Utiliza um ciclo que calcula o proximo movimento:
    *   - Sempre que ambos os motores terminarem o seu moviemento
    *   - Enquanto nao atingir a posição final
    */
    Point SubTarget;
    float m,b; /*Used to make the equation of a straight line*/  
    int flag=-1;

    /*Decide se divide a reta em x ou em y*/

    if(  abs( TargetPos.x - MyPos.x ) >= abs( TargetPos.y - MyPos.y) ){
        flag = FALSE; /* Dividir em x -> y = f(x)*/
        printf("--Equação em y = f(x)--\n\r");
    }
    else{
        flag = TRUE; /* Dividir em y -> x = F(y)*/
        printf("--Equação em x = f(y)--\n\r");
    }
    
    EquationOfaStraightLine(MyPos,TargetPos,&m,&b);
    printf("m : %d , b : %d", (int)(m*1000), (int)(b*1000));
    

    while( ( (MyPos.x < TargetPos.x) && (0==flag) ) || ( (MyPos.y < TargetPos.y) && (1==flag) ) ){

        if((0==motor1.step) && (0==motor2.step)){
            printf("FIRST WHILE\n\r");
            SubTarget = CalcSubTarget(flag,m,b,UP);
            Calc_move(SubTarget);
        }
        else{
            //printf("-- Motor1 steps left %d Motor2 steps left %d --\n\r", motor1.step, motor2.step);
        }
    }
    while( ( (MyPos.x > TargetPos.x) && (0==flag) ) || ( (MyPos.y > TargetPos.y) && (1==flag) ) ){
        
        if((0==motor1.step) && (0==motor2.step)){
            
            printf("SECOND WHILE\n\r");
            SubTarget = CalcSubTarget(flag,m,b,DOWN);
            Calc_move(SubTarget);
        }
        else{
            //printf("-- Motor1 steps left %d Motor2 steps left %d --\n\r", motor1.step, motor2.step);
        }   
    }

    return 1;
}

int main(void){
    
    hw_init();
    usart_init();
    printf_init();
    tc1_init();
    printf("\n\n\n\rScara arm starting (ง ͡ʘ ͜ʖ ͡ʘ)ง !\n\n\n\r");

    /* Testing the interrupt*/
    motor1.step=5000;
    motor1.t_between_step=1;
    motor1.dir=COUNTERCLOCKWISE;
    motor2.step=5000;
    motor2.t_between_step=3;
    motor2.dir=CLOCKWISE;
    
    /* Start - move motors till it touch the sensor then we knonw the position*/

    MyPos.x = 2;
    MyPos.y =1;
    InverseKinematics(Origin, MyPos, &MyPos.theta1, &MyPos.theta2);
    int x;
    /* TESTE DIRECTKINEMATICS
    DirectKinematics(MyPos.theta1,MyPos.theta2, &x, &y);
    printf(" %d %d \n\r ", (int)x,(int)y);
    */
    //printf("%d x %d y %d theta1 %d theta2", (int)(MyPos.x*1000), (int)(1000*MyPos.y), (int)(MyPos.theta1*1000), (int)(MyPos.theta2*1000));

    Point TargetPos = {2,2};
   InverseKinematics(Origin, TargetPos, &TargetPos.theta1, &TargetPos.theta2);
    _delay_ms(1000);
    Make_move(TargetPos);
    printf("lol");
    TargetPos.x = 5; 
    TargetPos.y = 2.5;
    _delay_ms(1000);
    Make_move(TargetPos);
    TargetPos.x = 10;
    TargetPos.y = 2.5;
    _delay_ms(1000);
  Make_move(TargetPos);

    while(1){
    
    }
  
}
