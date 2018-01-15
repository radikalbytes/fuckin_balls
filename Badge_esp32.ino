/*
Copyright 2017 Alfredo Prado Vega
@radikalbytes
http://www.radikalbytes.com
This work is licensed under the Creative Commons Attribution-ShareAlike 3.0
Unported License. To view a copy of this license, visit
http://creativecommons.org/licenses/by-sa/3.0/ or send a letter to
Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

The part of sand effect code is from Adafruit Industries
https://learn.adafruit.com/animated-led-sand/circuit-diagram and have their 
own license. Please check it at adafruit.com

*/
#define ESP32  // set sda and scl pins to 5,4 for ESP32 in MPU6050.h
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include <MPU6050.h>
#include "SSD1306.h" 
#include <Average.h>

// Include the UI lib
#include "OLEDDisplayUi.h"
//Include some images and fonts
#include "images2.h"
#include "Am_tw14.h"
#include "Am_tw16.h"



// Initialize the OLED display using Wire library
SSD1306  display(0x3c, 5, 4); // Lolin ESP-32 chinese
OLEDDisplayUi ui     ( &display );
//Initialize MPU6050
MPU6050 mpu;

//Acelerometter and Gyro variables
#define MPU6050_ACCEL_OFFSET_X 236
#define MPU6050_ACCEL_OFFSET_Y 129
#define MPU6050_ACCEL_OFFSET_Z 1076
#define MPU6050_GYRO_OFFSET_X  -24
#define MPU6050_GYRO_OFFSET_Y  -62
#define MPU6050_GYRO_OFFSET_Z  -6

#define HIST_SIZE 200

Average<float> ave(HIST_SIZE);

int16_t sensor_temperature;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float aax[HIST_SIZE];
float aay[HIST_SIZE];
float aaz[HIST_SIZE];

float agx[HIST_SIZE];
float agy[HIST_SIZE];
float agz[HIST_SIZE];

int16_t off_ax, off_ay, off_az;
int16_t off_gx, off_gy, off_gz;

//Variables de entorno
const int numeroBolitas = 2; //Particle number 
float friccion = -0.75;   //coeficiente de friccion con las paredes
                             //Es negativo para cambiar el 
                             //sentido del vector velocidad
                             //en el choque
float elasticidad = 0.2;  //Constante de elasticidad entre particulas 
int anchoDisplay = 128;  //Ancho matriz
int altoDisplay = 64;    //Alto mtriz
int factorReduccion = 1000;  //Factor de reduccion para suavizar movimientos

const float factorFiltro = 0.95;  //Factor de filtrado del acelerometro
//El factor de filtrado esta incluido en un filtro paso-bajo. Para el caso 
//de un sistema de particulas, es necesario un valor bajo de filtrado, ya
//que al usar un suavizado mayor, desaparecen los desplazamientos de las
//particulas con golpes o vibraciones de corta duraciion y el movimiento
//de estas pierde realismo

//Estructura de las particulas
struct particula {
 double posX;    //Posicion X
 double posY;    //Posicion Y
 float velX;    //Velocidad X
 float velY;    //Velocidad Y
 float masa;    //Masa de la particula
 byte radio;    //radio de la particula
};

//Crear particulas
particula bolita[numeroBolitas]={0,0,0,0};

//Valores en reposo para los ejes x,y medida anteriormente
//De ser necesario, se usara la rutina calibrado_sensor()
//para obtener nuevos valores de posicion en reposo
double LoX = 0;
double LoY = 0;

int timestep = 0;
Vector calibAccel;
Vector calibGyro;

//Buttons
int button = 0; //Button "Boot" from Lolin ESP32 oled
int buttonNext = 25; //Button at IO25
int buttonPrev = 16; //Button at IO16

// Size of screen
const int width = 128;
const int height = 64;

// Size of cells
const int cellSize = 1;

//Array definition
const int array_width = (width/cellSize);
const int array_height = (height/cellSize);

// Array of cells
int cells [array_width][array_height]={}; 
// Buffer to record the state of the cells and use this while changing the others in the interations
int cellsBuffer [array_width][array_height]={};
  
// How likely for a cell to be alive at start (in percentage)
float probabilityOfAliveAtStart = 15;

// Variables for timer
int interval = 100;
int lastRecordedTime = 0;
bool pause = 0;

// Colors for active/inactive cells
int alive = 1;
int dead = 0;

// Variables for Frame3
int mode_ = 0;

// Variables for Frame4
int _state = 0;
int _xx;
int _delay;
int _i;
int since;
int sirfredFrame;
#define left true
#define right false
bool sirfredDir ;
int sirfredX = 0;
int sirfredY = 40;

//variables frame5
int monsterX = 0;
int pause_frame5 = 100;
int _state5 = 0;
int randnumber;
int push = 0;

//sand effect
#define N_GRAINS     150 // Number of grains of sand
#define WIDTH_G        31 // Display width in pixels
#define HEIGHT_G       15 // Display height in pixels
// The 'sand' grains exist in an integer coordinate space that's 256X
// the scale of the pixel grid, allowing them to move and interact at
// less than whole-pixel increments.
#define MAX_X (WIDTH_G  * 256 - 1) // Maximum X coordinate in grain space
#define MAX_Y (HEIGHT_G * 256 - 1) // Maximum Y coordinate
struct Grain {
  int16_t  x,  y; // Position
  int16_t vx, vy; // Velocity
} grain[N_GRAINS];

uint32_t        prevTime   = 0;      // Used for frames-per-second throttle
uint8_t         backbuffer = 0;      // Index for double-buffered animation
bool        img[WIDTH_G * HEIGHT_G]; // Internal 'map' of pixels
                

float mean ( float * _array, int len ){
  ave.clear();
  for (int i = 0 ; i < len ; i++){
    ave.push(_array[i]);
  }
  return ave.mean();
}

float stddev ( float * _array, int len ){
  ave.clear();
  for (int i = 0 ; i < len ; i++){
    ave.push(_array[i]);
  }
  return ave.stddev();
}

void acel_calibration(){
      sensor_temperature = (mpu.readTemperature() + 12412) / 340;
      calibAccel = mpu.readRawAccel();
      calibGyro = mpu.readRawGyro();
      ax = calibAccel.XAxis; 
      ay = calibAccel.YAxis;
      az = calibAccel.ZAxis;
      gx = calibGyro.XAxis;
      gy = calibGyro.YAxis;
      gz = calibGyro.ZAxis;
      
      off_ax = mpu.getAccelOffsetX();
      off_ay = mpu.getAccelOffsetY();
      off_az = mpu.getAccelOffsetZ();
      off_gx = mpu.getGyroOffsetX();
      off_gy = mpu.getGyroOffsetY();
      off_gz = mpu.getGyroOffsetZ();

      aax[timestep] = ax;
      aay[timestep] = ay;
      aaz[timestep] = az;
      agx[timestep] = gx;
      agy[timestep] = gy;
      agz[timestep] = gz;
}

void accel_print_data()
{
    Serial.print("temp: ");
    Serial.print(sensor_temperature); Serial.print("|\t");
    Serial.print("timestep: ");
    Serial.print(timestep); Serial.print("|\t");
    Serial.print("a/g:|\t");
    Serial.print(ax); Serial.print("|\t");
    Serial.print(ay); Serial.print("|\t");
    Serial.print(az); Serial.print("|\t");
    Serial.print(gx); Serial.print("|\t");
    Serial.print(gy); Serial.print("|\t");
    Serial.print(gz); Serial.print("|\t");
    Serial.print(off_ax); Serial.print("|\t");
    Serial.print(off_ay); Serial.print("|\t");
    Serial.print(off_az); Serial.print("|\t");
    Serial.print(off_gx); Serial.print("|\t");
    Serial.print(off_gy); Serial.print("|\t");
    Serial.print(off_gz); Serial.print("|\t");
    Serial.print(mean(aax, timestep)); Serial.print("|\t");
    Serial.print(mean(aay, timestep)); Serial.print("|\t");
    Serial.print(mean(aaz, timestep)); Serial.print("|\t");
    Serial.print(mean(agx, timestep)); Serial.print("|\t");
    Serial.print(mean(agy, timestep)); Serial.print("|\t");
    Serial.print(mean(agz, timestep)); Serial.print("|\t");
    Serial.print(stddev(aax, timestep)); Serial.print("|\t");
    Serial.print(stddev(aay, timestep)); Serial.print("|\t");
    Serial.print(stddev(aaz, timestep)); Serial.print("|\t");
    Serial.print(stddev(agx, timestep)); Serial.print("|\t");
    Serial.print(stddev(agy, timestep)); Serial.print("|\t");
    Serial.print(stddev(agz, timestep));
    Serial.println("");
}

void cells_init(){
  // Initialization of cells
  for (int x=0; x<width/cellSize; x++) {
    for (int y=0; y<height/cellSize; y++) {
      float state = random (100);
      if (state > probabilityOfAliveAtStart) { 
        state = 0;
      }
      else {
        state = 1;
      }
      cells[x][y] = int(state); // Save state of each cell
    }
  }
}

int read_buttons(){
  int val;
  val = digitalRead(button);  // read input value
  if (val == LOW) {
    return 1;
  }
  val = digitalRead(buttonNext);  // read input value
  if (val == LOW) {
    return 2;
  }  
  val = digitalRead(buttonPrev);  // read input value
  if (val == LOW) {
    return 3;
  }  
  return 0;
}

void cells_iteration() { // When the clock ticks
  // Save cells to buffer (so we opeate with one array keeping the other intact)
  for (int x=0; x<width/cellSize; x++) {
    for (int y=0; y<height/cellSize; y++) {
      cellsBuffer[x][y] = cells[x][y];
    }
  }

  // Visit each cell:
  for (int x=0; x<width/cellSize; x++) {
    for (int y=0; y<height/cellSize; y++) {
      // And visit all the neighbours of each cell
      int neighbours = 0; // We'll count the neighbours
      for (int xx=x-1; xx<=x+1;xx++) {
        for (int yy=y-1; yy<=y+1;yy++) {  
          if (((xx>=0)&&(xx<width/cellSize))&&((yy>=0)&&(yy<height/cellSize))) { // Make sure you are not out of bounds
            if (!((xx==x)&&(yy==y))) { // Make sure to to check against self
              if (cellsBuffer[xx][yy]==1){
                neighbours ++; // Check alive neighbours and count them
              }
            } // End of if
          } // End of if
        } // End of yy loop
      } //End of xx loop
      // We've checked the neigbours: apply rules!
      if (cellsBuffer[x][y]==1) { // The cell is alive: kill it if necessary
        if (neighbours < 2 || neighbours > 3) {
          cells[x][y] = 0; // Die unless it has 2 or 3 neighbours
        }
      } 
      else { // The cell is dead: make it live if necessary      
        if (neighbours == 3 ) {
          cells[x][y] = 1; // Only if it has 3 neighbours
        }
      } // End of if
    } // End of y loop
  } // End of x loop
} // End of function

/****************************************************/
/*                                                  */
/*  Leer acelerometro y filtrar ruido con FPB       */
/*                                                  */
/****************************************************/
void lee_acelerometro(){
  //Leemos las entradas analogicas y le restamos el valor Zero  
  Vector rawAccel = mpu.readRawAccel();
  ax = rawAccel.XAxis;
  if (ax>32767) ax = ax - 65535;
  ay = rawAccel.YAxis;
  if (ay>32767) ay = ay - 65535;
  
  //Filtro paso Bajo
  LoX = ax * factorFiltro + (LoX * (1.0 - factorFiltro));
  LoY = ay * factorFiltro + (LoY * (1.0 - factorFiltro));
}

/****************************************************/
/*                                                  */
/*        Colisiones entre particulas               */
/*                                                  */
/****************************************************/
// Vamos a ir comprobando la distancia entre particulas. En caso de ser
// d < 1 calculamos los nuevos vectores de velocidad resultantes usando
// trigonometria y vectores.
// El calculo es: 
// particula 1 con la 2,3,4,5,6,...,n
// particula 2 con la 3,4,5,6,....,n (con la 1 ya esta calculado)
// .....
// particula n-1 con la n

void colision_bolas(){
  for (int o = 0;o < numeroBolitas-1; o++){ //particula a comparar
      for (int i = o + 1; i < numeroBolitas; i++) { //resto de particulas
          float x1 = bolita[i].posX/factorReduccion;
          float x2 = bolita[o].posX/factorReduccion;
          float y1 = bolita[i].posY/factorReduccion;
          float y2 = bolita[o].posY/factorReduccion;
          //calculamos la distancia entre particulas usando Pitagoras
          float dx = x1-x2;
          float dy = y1-y2;
          float distancia = sqrt(dx*dx + dy*dy);
          float minimaDistancia = bolita[i].radio + bolita[o].radio;
          //Si colisionan, calculamos los nuevos vectores de velocidad resultantes
          if (distancia < minimaDistancia) { //colision
            float angle = atan2(dy, dx);  //Angulo de la colision
            //Calculo de la nueva posicion
            float targetX = x1 + cos(angle) * minimaDistancia ;
            float targetY = y1 + sin(angle) * minimaDistancia ;
            //Desplazamiento producido y aplicacion de un coeficiente de
            //reduccion por la friccion entre particulas
            float ax = (targetX - x2) * elasticidad;
            float ay = (targetY - y2) * elasticidad;
            Serial.print (ax);
            Serial.print("\t");
            Serial.println(ay);
            //Ajuste de las velocidades en base al desplazamiento generado
            //La que impacta reduce su velocidad y la impactada la incrementa
            //Habeis visto las bolas de billar cuando chocan???
            bolita[o].velX -= (ax*factorReduccion*bolita[o].masa);
            bolita[o].velY -= (ay*factorReduccion*bolita[o].masa);
            bolita[i].velX += (ax*factorReduccion*bolita[i].masa);
            bolita[i].velY += (ay*factorReduccion*bolita[i].masa);
          }
       } 
  }
}

void colision2 (particula *_bolita){
  for (int o = 0;o < numeroBolitas-1; o++){ //particula a comparar
    for (int i = o + 1; i < numeroBolitas; i++) { //resto de particulas
      //calculamos la distancia entre particulas usando Pitagoras
      float dx = _bolita[i].posX/factorReduccion - _bolita[o].posX/factorReduccion;
      float dy = _bolita[i].posY/factorReduccion - _bolita[o].posY/factorReduccion;
      float distancia = sqrt(dx*dx + dy*dy);
      float minimaDistancia = _bolita[i].radio + _bolita[o].radio;
      //Si colisionan, calculamos los nuevos vectores de velocidad resultantes
      if (distancia < minimaDistancia) { //colision
        float angle = atan2(dy, dx);  //Angulo de la colision
        //Calculo de la nueva posicion
        float targetX = _bolita[o].posX/factorReduccion + cos(angle) ;
        float targetY = _bolita[o].posY/factorReduccion + sin(angle) ;
        //Desplazamiento producido y aplicacion de un coeficiente de
        //reduccion por la friccion entre particulas
        float ax = (targetX - _bolita[i].posX/factorReduccion) * friccion;
        float ay = (targetY - _bolita[i].posY/factorReduccion) * friccion;
        //Ajuste de las velocidades en base al desplazamiento generado
        //La que impacta reduce su velocidad y la impactada la incrementa
        //Habeis visto las bolas de billar cuando chocan???
        bolita[o].velX -= ax*bolita[o].masa*factorReduccion;
        bolita[o].velY -= ay*bolita[o].masa*factorReduccion;
        _bolita[i].velX += ax*bolita[i].masa*factorReduccion;
        _bolita[i].velY += ay*bolita[i].masa*factorReduccion;
      }
    } 
  }
}
/****************************************************/
/*                                                  */
/*           Posicionado de particulas              */
/*                                                  */
/****************************************************/
void mueve_bolas(){
  //Actualizamos posiciones de particulas 
  for (int o = 0;o < numeroBolitas; o++){
    //Colisiones con los limites del ejeX 
    bolita[o].velX += (LoX/4)*bolita[o].masa; //Reducimos a un 0.25 el valor del acelerometro
                                              //y variamos la velocidad respecto a la masa
    bolita[o].posX += bolita[o].velX;         // Calculamos nueva posicion ejeX
    //Comprobamos colisiones con los extremos de la matriz
    if ((bolita[o].posX - (bolita[o].radio*factorReduccion)) < 0) {        //Comprobamos si hay rebote en punto X=0
        bolita[o].posX = (bolita[o].radio*factorReduccion);          //Posicionamos en X=0
        bolita[o].velX = friccion*bolita[o].velX; //Invertimos la direccion del vector de 
                                                     //velocidad y le aplicamos un coeficiente
                                                     //de reduccion (friccion)
    }
    //rebote en el otro extremo del ejeX procedemos igual
    else if ( ((bolita[o].posX)+(bolita[o].radio*factorReduccion)) > ((anchoDisplay-1)*factorReduccion) ) { 
        bolita[o].posX = ((anchoDisplay-1)*factorReduccion) - (bolita[o].radio*factorReduccion);
        bolita[o].velX = friccion*bolita[o].velX;
    }
    
    //Colisiones con los limites del ejeY
    //Procedemos igual que con el eje X
    bolita[o].velY += (LoY/4)*bolita[o].masa;
    bolita[o].posY += bolita[o].velY;
    
    if ((bolita[o].posY - (bolita[o].radio*factorReduccion)) < 0) {        
        bolita[o].posY = (bolita[o].radio*factorReduccion);
        bolita[o].velY = friccion*bolita[o].velY;
    }
    else if ( ((bolita[o].posY)+(bolita[o].radio*factorReduccion)) > ((altoDisplay-1)*factorReduccion) ) { 
        bolita[o].posY = ((altoDisplay-1)*factorReduccion) - (bolita[o].radio*factorReduccion);
        bolita[o].velY = friccion*bolita[o].velY;
    }
    
  }
}

/**
 * Sand Effect 
 * Algorithm code from Adafruit Industries
 * https://learn.adafruit.com/animated-led-sand/circuit-diagram
 */
void mueve_arena(){
  int16_t aax, aay, aaz; 
  // Read accelerometer...
  Vector rawAccel = mpu.readRawAccel();
  aax = rawAccel.XAxis;
  if (aax>32767) aax = aax - 65535;
  aay = rawAccel.YAxis;
  if (aay>32767) aay = aay - 65535;
  aaz = rawAccel.ZAxis;
  if (aaz>32767) aaz = aaz - 65535;
  
  aax = aax / 256;      // Transform accelerometer axes
  aay =  aay / 256;      // to grain coordinate space
  aaz = abs(aaz) / 2048; // Random motion factor
  aaz = (aaz >= 3) ? 1 : 4 - aaz;      // Clip & invert
  aax -= aaz;                         // Subtract motion factor from X, Y
  aay -= aaz;
  int16_t az2 = aaz * 2 + 1;         // Range of random motion to add back in

  // ...and apply 2D accel vector to grain velocities...
  int32_t v2; // Velocity squared
  float   v;  // Absolute velocity
  for(int i=0; i<N_GRAINS; i++) {
    grain[i].vx += aax + random(az2); // A little randomness makes
    grain[i].vy += aay + random(az2); // tall stacks topple better!
    // Terminal velocity (in any direction) is 256 units -- equal to
    // 1 pixel -- which keeps moving grains from passing through each other
    // and other such mayhem.  Though it takes some extra math, velocity is
    // clipped as a 2D vector (not separately-limited X & Y) so that
    // diagonal movement isn't faster
    v2 = (int32_t)grain[i].vx*grain[i].vx+(int32_t)grain[i].vy*grain[i].vy;
    if(v2 > 65536) { // If v^2 > 65536, then v > 256
      v = sqrt((float)v2); // Velocity vector magnitude
      grain[i].vx = (int)(256.0*(float)grain[i].vx/v); // Maintain heading
      grain[i].vy = (int)(256.0*(float)grain[i].vy/v); // Limit magnitude
    }
  }

  // ...then update position of each grain, one at a time, checking for
  // collisions and having them react.  This really seems like it shouldn't
  // work, as only one grain is considered at a time while the rest are
  // regarded as stationary.  Yet this naive algorithm, taking many not-
  // technically-quite-correct steps, and repeated quickly enough,
  // visually integrates into something that somewhat resembles physics.
  // (I'd initially tried implementing this as a bunch of concurrent and
  // "realistic" elastic collisions among circular grains, but the
  // calculations and volument of code quickly got out of hand for both
  // the tiny 8-bit AVR microcontroller and my tiny dinosaur brain.)

  uint8_t        i_i, bytes;
  int16_t        delta,oldidx,newidx;
  int16_t        newx, newy;
  //const uint8_t *ptr = remap;

  for(i_i=0; i_i<N_GRAINS; i_i++) {
    newx = grain[i_i].x + grain[i_i].vx; // New position in grain space
    newy = grain[i_i].y + grain[i_i].vy;
    if(newx > MAX_X) {               // If grain would go out of bounds
      newx         = MAX_X;          // keep it inside, and
      grain[i_i].vx /= -1.2;             // give a slight bounce off the wall
    } else if(newx < 0) {
      newx         = 0;
      grain[i_i].vx /= -1.2;
    }
    if(newy > MAX_Y) {
      newy         = MAX_Y;
      grain[i_i].vy /= -1.2;
    } else if(newy < 0) {
      newy         = 0;
      grain[i_i].vy /= -1.2;
    }

    oldidx = (grain[i_i].y/256) * WIDTH_G + (grain[i_i].x/256); // Prior pixel #
    newidx = (newy      /256) * WIDTH_G + (newx      /256); // New pixel #
    if((oldidx != newidx) && // If grain is moving to a new pixel...
        img[newidx]) {       // but if that pixel is already occupied...
      delta = abs(newidx - oldidx); // What direction when blocked?
      if(delta == 1) {            // 1 pixel left or right)
        newx         = grain[i_i].x;  // Cancel X motion
        grain[i_i].vx /= -2;          // and bounce X velocity (Y is OK)
        newidx       = oldidx;      // No pixel change
      } else if(delta == WIDTH_G) { // 1 pixel up or down
        newy         = grain[i_i].y;  // Cancel Y motion
        grain[i_i].vy /= -2;          // and bounce Y velocity (X is OK)
        newidx       = oldidx;      // No pixel change
      } else { // Diagonal intersection is more tricky...
        // Try skidding along just one axis of motion if possible (start w/
        // faster axis).  Because we've already established that diagonal
        // (both-axis) motion is occurring, moving on either axis alone WILL
        // change the pixel index, no need to check that again.
        if((abs(grain[i_i].vx) - abs(grain[i_i].vy)) >= 0) { // X axis is faster
          newidx = (grain[i_i].y / 256) * WIDTH_G + (newx / 256);
          if(!img[newidx]) { // That pixel's free!  Take it!  But...
            newy         = grain[i_i].y; // Cancel Y motion
            grain[i_i].vy /= -2;         // and bounce Y velocity
          } else { // X pixel is taken, so try Y...
            newidx = (newy / 256) * WIDTH_G + (grain[i_i].x / 256);
            if(!img[newidx]) { // Pixel is free, take it, but first...
              newx         = grain[i_i].x; // Cancel X motion
              grain[i_i].vx /= -2;         // and bounce X velocity
            } else { // Both spots are occupied
              newx         = grain[i_i].x; // Cancel X & Y motion
              newy         = grain[i_i].y;
              grain[i_i].vx /= -2;         // Bounce X & Y velocity
              grain[i_i].vy /= -2;
              newidx       = oldidx;     // Not moving
            }
          }
        } else { // Y axis is faster, start there
          newidx = (newy / 256) * WIDTH_G + (grain[i_i].x / 256);
          if(!img[newidx]) { // Pixel's free!  Take it!  But...
            newx         = grain[i_i].x; // Cancel X motion
            grain[i_i].vy /= -2;         // and bounce X velocity
          } else { // Y pixel is taken, so try X...
            newidx = (grain[i_i].y / 256) * WIDTH_G + (newx / 256);
            if(!img[newidx]) { // Pixel is free, take it, but first...
              newy         = grain[i_i].y; // Cancel Y motion
              grain[i_i].vy /= -2;         // and bounce Y velocity
            } else { // Both spots are occupied
              newx         = grain[i_i].x; // Cancel X & Y motion
              newy         = grain[i_i].y;
              grain[i_i].vx /= -2;         // Bounce X & Y velocity
              grain[i_i].vy /= -2;
              newidx       = oldidx;     // Not moving
            }
          }
        }
      }
    }
    grain[i_i].x  = newx; // Update grain position
    grain[i_i].y  = newy;
    img[oldidx] = 0;    // Clear old spot (might be same as new, that's OK)
    img[newidx] = 1;  // Set new spot
  }

}

// Overlay Screen
void msOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {

}

void msOverlay2(OLEDDisplay *display, OLEDDisplayUiState* state){

  
}

void drawFrame1(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  // draw an xbm image.
  // Please note that everything that should be transitioned
  // needs to be drawn relative to x and y
  display->setColor(WHITE); 
  display->drawXbm(x + 0, y + 0, Gol_width, Gol_height, Gol_bits);
  switch (read_buttons()){
    case 1:

    break;
    
    case 2:
      ui.nextFrame();
    break;

    case 3:
      _state = 0;
      monsterX = 0;
      ui.previousFrame();
    break;
  }
}

void drawFrame2(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  bool pushed_1 = HIGH;
  switch (read_buttons()){
    case 1:
      cells_init();
      pushed_1 = LOW;
    break;
    
    case 2:
      ui.nextFrame();
    break;

    case 3:
      ui.previousFrame();
    break;
  }
  display->setColor(WHITE); 
  //Draw grid
  for (int xx=0; xx<width/cellSize; xx++) {
    for (int yy=0; yy<height/cellSize; yy++) {
      if (cells[xx][yy]==1) 
        display->fillRect ((xx*cellSize)+x, (yy*cellSize)+y, cellSize, cellSize);
    }
  }
    if (pushed_1 == LOW) {
      display->setTextAlignment(TEXT_ALIGN_CENTER);
      display->setColor(WHITE);
      display->setFont(ArialMT_Plain_16);
      display->drawString(64, 0, "@radikalbytes");
      display->drawString(64, 20, "Game of Life");
      display->drawString(64, 40, "2017");
    }
  // Iterate if timer ticks
  if (millis()-lastRecordedTime>interval) {
    cells_iteration();
    lastRecordedTime = millis();
  }
}

void drawFrame3(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  Vector rawAccel = mpu.readRawAccel();
  
  ax = rawAccel.XAxis;
  if (ax>32767) ax = ax - 65535;
  
  ay = rawAccel.YAxis;
  if (ay>32767) ay = ay - 65535;
  
  az = rawAccel.ZAxis;
  
  Vector normAccel = mpu.readNormalizeAccel();
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setColor(WHITE);
  display->setFont(ArialMT_Plain_16);
  display->drawString(64+x, 0+y, "MPU6050 Axis");
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  switch (mode_) {
    case 0:
      display->drawString(0+x, 20+y, "Xraw: " + String(rawAccel.XAxis) );
      display->drawString(0+x, 30+y, "Yraw: " + String(rawAccel.YAxis));
      display->drawString(0+x, 40+y, "Zraw: " + String(rawAccel.ZAxis));
    break;
    case 1:
      display->drawString(0+x, 20+y, "Xnorm: " + String(normAccel.XAxis) );
      display->drawString(0+x, 30+y, "Ynorm: " + String(normAccel.YAxis));
      display->drawString(0+x, 40+y, "Znorm: " + String(normAccel.ZAxis));
    break;
    case 2:
      display->drawString(0+x, 20+y, "Xacel: " + String(ax));
      display->drawString(0+x, 30+y, "Yacel: " + String(ay));
      display->drawString(0+x, 40+y, "Zacel: " + String(az));
    break;
  }
  switch (read_buttons()){
    case 1:
      mode_++;
      if (mode_ == 3) mode_ = 0;
      delay(200);
    break;
    
    case 2:
      ui.nextFrame();
    break;

    case 3:
      ui.previousFrame();
    break;
  }
  
}

void drawFrame4(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  int _ready;
 // Serial.print("_state:");
 // Serial.println(_state);
  switch (_state){
    case 0:
      display->setTextAlignment(TEXT_ALIGN_CENTER);
      display->setColor(WHITE);
      display->setFont(ArialMT_Plain_16);
      display->drawStringMaxWidth(64+x, 0+y, 128, "MPU6050 Calibration");
      display->setFont(ArialMT_Plain_10);
      display->drawString(64+x, 40+y, "Put PCB on a flat surface");
      display->drawString(64+x, 50+y, "If OK... push!!!");
      switch (read_buttons()){
        case 1:
          if (_state == 0) {
            timestep = 0;   
            _xx = 100;
            _i = 0;
            lastRecordedTime = millis();
            _state++;
          }
        break;
    
        case 2:
          _state5 = 0;
          monsterX = 48;
          since=millis();
          ui.nextFrame();
        break;

        case 3:
          ui.previousFrame();
        break;
      }
    break;
    
    case 1:
      display->setFont(ArialMT_Plain_16);
      display->drawStringMaxWidth(64+x, 0+y, 128, "MPU6050 Calibration");
      display->setFont(ArialMT_Plain_10);
      display->drawString(64+x, 40+y, "Put PCB on a flat surface");
      display->drawProgressBar(10+x,55+y,100,8,_xx);
      if (millis()-lastRecordedTime>30) {
        _xx--;
        if(_xx == 0){
          _i = 0;
          _state++;
        }
        lastRecordedTime = millis();
      }
    break;
    
    case 2:
      acel_calibration();
      display->setFont(ArialMT_Plain_16);
      display->drawStringMaxWidth(64+x, 0+y, 128, "MPU6050 Warming up...");
      display->setFont(ArialMT_Plain_10);
      display->drawString(64+x, 40+y, "Put PCB on a flat surface");
      display->drawProgressBar(10+x,55+y,100,8,_i/2);
      if (millis() - lastRecordedTime > 50) {
        //ººaccel_print_data();
        lastRecordedTime = millis();
        timestep++;
        _i++;
        if (timestep > 199) timestep = 0;
        if (_i == 200){
          _i=0;
          timestep = 0;
          since = millis();
          sirfredX = 0;
          sirfredY = 33;
          sirfredDir = right;
          sirfredFrame = 0;
          _state++;
        }
      }
    break;

    case 3:
      
      display->setFont(ArialMT_Plain_16);
      display->drawStringMaxWidth(64, 0, 128, "Setting values...");
      if (sirfredDir == 0) {
        if (sirfredX > 100) display->drawXbm(sirfredX, sirfredY, sirfred_width, sirfred_height, sf_derrapeder);
        else{
          if ((sirfredFrame == 0) || (sirfredFrame == 1)){
            display->drawXbm(sirfredX, sirfredY, sirfred_width, sirfred_height, sf_der1);
          }
          else display->drawXbm(sirfredX, sirfredY, sirfred_width, sirfred_height, sf_der2);  
        }  
      }
      else {
        if (sirfredX < 15) display->drawXbm(sirfredX, sirfredY, sirfred_width, sirfred_height, sf_derrapeizq);
        else{
          if ((sirfredFrame == 0) || (sirfredFrame == 1)) {
            display->drawXbm(sirfredX, sirfredY, sirfred_width, sirfred_height, sf_izq1);
          }
          else display->drawXbm(sirfredX, sirfredY, sirfred_width, sirfred_height, sf_izq2); 
        }
      }
      
      display->setColor(BLACK);
      display->drawXbm(48 ,33 ,barandilla_width ,barandilla_height, barandilla_mask);
      display->setColor(WHITE);
      display->drawXbm(48 ,33 ,barandilla_width ,barandilla_height, barandilla);
      display->drawXbm(0 ,56 ,suelo_width ,suelo_height, suelo);

      
      if (millis() - since > 40) {
        if ((sirfredDir == right) && (sirfredX > 112)){
          sirfredDir = left;
        }
        if ((sirfredDir == left) && (sirfredX < 1)){
          sirfredDir = right;
        }
        sirfredFrame++;
        if (sirfredFrame == 4) sirfredFrame = 0;
        Serial.print("SirfredFrame: ");
        Serial.println(sirfredFrame);
        switch (sirfredFrame){
          case 0:
            if (sirfredDir == right){
              sirfredX++;
            }
            else sirfredX--;
          break;

          case 1:
          case 2:
          case 3:
            if (sirfredDir == right){
              sirfredX=sirfredX+2;
            }
            else sirfredX=sirfredX-2;
          break;
          
        }
        since = millis();
      }
      
      if (millis() - lastRecordedTime > 200) {
        acel_calibration();
        if (ax > 0) off_ax--; else if (ax < 0) off_ax++;
        if (ay > 0) off_ay--; else if (ay < 0) off_ay++;
        if (az > 16384) off_az--; else if (az < 16384) off_az++;

        if (gx > 0) off_gx--; else if (gx < 0) off_gx++;
        if (gy > 0) off_gy--; else if (gy < 0) off_gy++;
        if (gz > 0) off_gz--; else if (gz < 0) off_gz++;

        mpu.setAccelOffsetX(off_ax);
        mpu.setAccelOffsetY(off_ay);
        mpu.setAccelOffsetZ(off_az);
        mpu.setGyroOffsetX(off_gx);
        mpu.setGyroOffsetY(off_gy);
        mpu.setGyroOffsetZ(off_gz);

        //accel_print_data();   
        timestep++;
        if (timestep >= HIST_SIZE) {
          timestep = 0;
          _state = 0;
        }
        lastRecordedTime = millis();
      }
    break;

    
    
  }
}

void drawFrame5(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
      display->setTextAlignment(TEXT_ALIGN_CENTER);
      display->setColor(WHITE);
      switch (read_buttons()){
        case 1:
          
        break;
    
        case 2:
          ui.nextFrame();
          since = millis();
        break;

        case 3:
          ui.previousFrame();
        break;
      }//End switch

      if (millis()-since>100){
        if(_state5==8) _state5--;
        else if(_state5==10) _state5--;
        else _state5++;
        since = millis();
        randnumber = random(100);
        if (randnumber % 20 == 0) _state5 = 0;
      }//End if
      push = 0;
      switch(_state5){
        case 0: case 6:    
          display->drawXbm(monsterX+x, 20+y, monster_width, monster_height, monster1);
        break;
        case 1: case 5:
          display->drawXbm(monsterX+x, 20+y, monster_width, monster_height, monster2);
        break;
        case 2:    
          display->drawXbm(monsterX+x, 20+y, monster_width, monster_height, monster3);
        break;
        case 3: case 4:
          display->drawXbm(monsterX+x, 20+y, monster_width, monster_height, monster4);
          push = 1;
        break;
        case 7:
          monsterX++;
          display->drawXbm(monsterX+x, 20+y, monstercamina_width, monstercamina_height, monsterder1);
        break;
        case 8:
          monsterX++;
          display->drawXbm(monsterX+x, 19+y, monstercamina_width, monstercamina_height, monsterder2);
          if(monsterX>=100) _state5 = 9;
        break;
        case 9:
          monsterX--;
          display->drawXbm(monsterX+x, 20+y, monstercamina_width, monstercamina_height, monsteriz1);
        break;
        case 10:
          monsterX--;
          display->drawXbm(monsterX+x, 19+y, monstercamina_width, monstercamina_height, monsteriz2);
          if(monsterX<=0) _state5 = 7;
        break;
            
      }//End switch
      display->setFont(AmericanTypewriter_Plain_16);
      display->drawStringMaxWidth(64+x, 1+y-push, 128, "Fuckin balls");
      display->setFont(AmericanTypewriter_Plain_14);
      
      display->drawString(64+x, 50+y+push, "Push...");
}

void drawFrame6(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {;
      switch (read_buttons()){
        case 1:
          
        break;
    
        case 2:
          since=millis();
          ui.nextFrame();
        break;

        case 3:
          ui.previousFrame();
        break;
      }//End switch
     //display->setTextAlignment(TEXT_ALIGN_LEFT);
     display->setColor(WHITE);
     display->drawRect(0+x,0+y,128,64);
     display->setFont(ArialMT_Plain_10);
     //display->drawString(20+x, 20+y, "Xacel: " + String(LoX));
     //display->drawString(20+x, 30+y, "Yacel: " + String(LoY));
     for (int o = 0;o < numeroBolitas; o++){
       //display->setPixel(bolita[o].posX/factorReduccion+1+x, bolita[o].posY/factorReduccion+1+y);
      // display->drawString(0+x, 20+y, "X: " + String((bolita[o].posX/factorReduccion)+1-bolita[o].radio+x)+" "+String((bolita[o].posX/factorReduccion+1)));
      // display->drawString(0+x, 30+y, "Y: " + String((bolita[o].posY/factorReduccion)+1-bolita[o].radio+y)+" "+String((bolita[o].posY/factorReduccion+1)));
       switch (bolita[o].radio){
         case 4:
           display->drawXbm((bolita[o].posX/factorReduccion)-bolita[o].radio+x, (bolita[o].posY/factorReduccion)-bolita[o].radio+y, bola8x8_width, bola8x8_height, bola8x8);
         break;
         case 7:
           display->drawXbm((bolita[o].posX/factorReduccion)-bolita[o].radio+x+1, (bolita[o].posY/factorReduccion)-bolita[o].radio+y+1, bola13x13_width, bola13x13_height, bola13x13);
         break;
       }// End switch
     }
     if (millis()-since>30){  
       lee_acelerometro();  // Lee acelerometro
       colision_bolas();
       //colision2(bolita);    // Calculo de colisiones
       mueve_bolas();       // Movimiento de las particulas
       since = millis();
     }
}

void drawFrame7(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {;
      switch (read_buttons()){
        case 1:
          Serial.println(sizeof(img));
          for(int qwerty=0;qwerty<(sizeof(img));qwerty++){
            if (img[qwerty]) Serial.print("1");
            else Serial.print("0");
            if (qwerty%WIDTH_G==0) Serial.println();
          }
          Serial.println();
          delay(60);
        break;
    
        case 2:
          ui.nextFrame();
        break;

        case 3:
          ui.previousFrame();
        break;
      } //End switch
     display->setColor(WHITE);
     /*
     for (int16_t rr=0;rr<sizeof(img);rr++){
      if(img[rr]) display->drawRect( (rr%WIDTH_G)*4 +x, (rr/WIDTH_G)*4 +y,4,4);
      Serial.print((rr%WIDTH_G)*4);
      Serial.print(",");
      Serial.println((rr/WIDTH_G)*4);
      
     }*/
     
     for (int i=0;i<N_GRAINS;i++){
      //display->setPixel(grain[i].x/256 +54+ x,grain[i].y/256 +22+ y);
      display->drawRect(grain[i].x/64 +x,grain[i].y/64+y,4,4);
     }
     
     //if (millis()-since>30){ 
       mueve_arena();
       since=millis();
     //}

     display->setFont(ArialMT_Plain_10);
     //display->drawString(0+x, 20+y, "X: " + String(grain[0].x) );
     //display->drawString(0+x, 40+y, "Y: " + String(grain[0].y) );
     
}

// This array keeps function pointers to all frames
// frames are the single views that slide in
FrameCallback frames[] = { drawFrame1, drawFrame2, drawFrame3, drawFrame4, drawFrame5, drawFrame6, drawFrame7};

// how many frames are there?
int frameCount = 7;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = { msOverlay, msOverlay2 };
int overlaysCount = 0;

void setup() {
  //Button mode input
  pinMode(button, INPUT);
  pinMode(buttonNext, INPUT_PULLUP);
  pinMode(buttonPrev, INPUT_PULLUP);
  
	// The ESP is capable of rendering 60fps in 80Mhz mode
	// but that won't give you much time for anything else
	// run it in 160Mhz mode or just set it to 30 fps
  ui.setTargetFPS(60);

  //disable indicator on screen
  ui.disableAllIndicators();

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  ui.setFrames(frames, frameCount);

  // Add overlays
  ui.setOverlays(overlays, overlaysCount);

  // Initialising the UI will init the display too.
  ui.init();

  //Disable auto transition between frames
  ui.disableAutoTransition();
  
  display.flipScreenVertically();
  Serial.begin(115200);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    //yield(); // Do (almost) nothing -- yield to allow ESP32 background functions
    delay(500);
  }

  
//Posicionado de las particulas con velocidades=0
//Se asigna una masa aleatoria de entre 1<=m<=2
for (int o = 0;o < numeroBolitas; o++){
  bolita[o].posX = random(anchoDisplay*factorReduccion);
  bolita[o].posY = random(altoDisplay*factorReduccion);
  bolita[o].velX = 0;
  bolita[o].velY = 0;
  if(o%3 == 0) bolita[o].radio = 7;
  else bolita[o].radio = 4;
  bolita[o].masa = bolita[o].radio / 4;
}


  // If you want, you can set accelerometer offsets
  
    mpu.setAccelOffsetX(MPU6050_ACCEL_OFFSET_X);
    mpu.setAccelOffsetY(MPU6050_ACCEL_OFFSET_Y);
    mpu.setAccelOffsetZ(MPU6050_ACCEL_OFFSET_Z);
    mpu.setGyroOffsetX(MPU6050_GYRO_OFFSET_X);
    mpu.setGyroOffsetY(MPU6050_GYRO_OFFSET_Y);
    mpu.setGyroOffsetZ(MPU6050_GYRO_OFFSET_Z);
  
  Serial.println("MPU6050 Initalized");
  cells_init(); 
uint8_t iii, jjj, bytes;
memset(img, 0, sizeof(img)); // Clear the img[] array
for(iii=0; iii<N_GRAINS; iii++) {  // For each sand grain...
    do {
      grain[iii].x = random(WIDTH_G  * 256); // Assign random position within
      grain[iii].y = random(HEIGHT_G * 256); // the 'grain' coordinate space
      // Check if corresponding pixel position is already occupied...
      for(jjj=0; (jjj<iii) && (((grain[iii].x / 256) != (grain[jjj].x / 256)) ||
                         ((grain[iii].y / 256) != (grain[jjj].y / 256))); jjj++);
    } while(jjj < iii); // Keep retrying until a clear spot is found
    img[(grain[iii].y / 256) * WIDTH_G + (grain[iii].x / 256)] = 1; // Mark it
    grain[iii].vx = grain[iii].vy = 0; // Initial velocity is zero
  }
  
}


void loop() {
  int remainingTimeBudget = ui.update();

  if (remainingTimeBudget > 0) {
    // You can do some work here
    // Don't do stuff if you are below your
    // time budget.
    delay(remainingTimeBudget);
  }
}
