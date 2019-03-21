/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib> 
#include <cstdio>


/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "bluebatterysensor.h"
#include "redbatterysensor.h"
#include "encodersensor.h"
#include "compasssensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"

extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

char* mapDraw =
"%%%%%%%%%%%%%%%%%%%%"
"%############%#####%"
"%############%#####%"
"%##%%%%%%%%##%##%##%"
"%##%######%#####%##%"
"%##%######%#####%##%"
"%##%##%%%%%%%%%%%##%"
"%##%###############%"
"%##%###############%"
"%##%##%%%%%%%%#####%"
"%##%##%######%%%%%%%"
"%##%###############%"
"%##%#####%%########%"
"%##%%%%%%%%%%%#####%"
"%##%##%############%"
"%##%##%############%"
"%##%##%#####%%%%%##%"
"%#####%#########%##%"
"%#####%#########%##%"
"%%%%%%%%%%%%%%%%%%%%";

//"%%%%%%%%%%%%%%%%%%%%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%%%%%%%%%%%%%%%%%%%%";

//"%%%%%%%%%%%%%%%%%%%%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%%%%%%%%%%%%%%#####%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%%%%%%%%%%%%%%%%%%%%";

const int mapGridX          = 20;
const int mapGridY          = 20;
double    mapLengthX        = 3.0;
double    mapLengthY        = 3.0;
double    robotStartGridX   = 11; 
double    robotStartGridY   = 18;
double    robotEndGridX     = 15;
double    robotEndGridY     = 1;

const   int n=mapGridX; // horizontal size of the map
const   int m=mapGridY; // vertical size size of the map
static  int map[n][m];
static  int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
static  int open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
static  int dir_map[n][m]; // map of directions
const   int dir=8; // number of possible directions to go at any position
// if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
// if dir==8
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

#define ERROR_DIRECTION 0.05 
#define ERROR_POSITION  0.01
#define SPEED           300

class node
{
  // current position
  int xPos;
  int yPos;
  // total distance already travelled to reach the node
  int level;
  // priority=level+remaining distance estimate
  int priority;  // smaller: higher priority

  public:
  node(int xp, int yp, int d, int p) 
  {xPos=xp; yPos=yp; level=d; priority=p;}

  int getxPos() const {return xPos;}
  int getyPos() const {return yPos;}
  int getLevel() const {return level;}
  int getPriority() const {return priority;}

  void updatePriority(const int & xDest, const int & yDest)
  {
    priority=level+estimate(xDest, yDest)*10; //A*
  }

  // give better priority to going strait instead of diagonally
  void nextLevel(const int & i) // i: direction
  {
    level+=(dir==8?(i%2==0?10:14):10);
  }

  // Estimation function for the remaining distance to the goal.
  const int & estimate(const int & xDest, const int & yDest) const
  {
    static int xd, yd, d;
    xd=xDest-xPos;
    yd=yDest-yPos;         

    // Euclidian Distance
    d=static_cast<int>(sqrt(xd*xd+yd*yd));

    // Manhattan distance
    //d=abs(xd)+abs(yd);

    // Chebyshev distance
    //d=max(abs(xd), abs(yd));

    return(d);
  }
};

/******************************************************************************/
/******************************************************************************/

// Determine priority (in the priority queue)
bool operator < ( const node & a, const node & b )
{
  return a.getPriority() > b.getPriority();
}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	/* Set Write to File */
	m_nWriteToFile = n_write_to_file;

	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set Blue light Sensor */
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
	/* Set Red light Sensor */
	m_seRedLight = (CRealRedLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set battery Sensor */
	m_seBattery = (CBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BATTERY);
	/* Set blue battery Sensor */
	m_seBlueBattery = (CBlueBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BLUE_BATTERY);
	/* Set red battery Sensor */
	m_seRedBattery = (CRedBatterySensor*) m_pcEpuck->GetSensor (SENSOR_RED_BATTERY);
	/* Set encoder Sensor */
	m_seEncoder = (CEncoderSensor*) m_pcEpuck->GetSensor (SENSOR_ENCODER);
  m_seEncoder->InitEncoderSensor(m_pcEpuck);
	/* Set compass Sensor */
	m_seCompass = (CCompassSensor*) m_pcEpuck->GetSensor (SENSOR_COMPASS);

  m_nState=0;
  m_nPathPlanningStops=0;

  PathPlanning();
}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller()
{
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{


	/* FASE 1: LECTURA DE SENSORES */

	/* Leer Sensores de Proximidad */
	double* prox = m_seProx->GetSensorReading(m_pcEpuck);
	double* ground = m_seGround->GetSensorReading(m_pcEpuck);
	/* Leer Encoder */
	double* encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
	/* Leer Compass */
	double* compass = m_seCompass->GetSensorReading(m_pcEpuck);

	
	/* FASE 2: CONTROLADOR */
	
	/* Inicio Incluir las ACCIONES/CONTROLADOR a implementar */
	printf("PROX: ");
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", prox[i]);
	}
  printf("\n");
	/* Fin: Incluir las ACCIONES/CONTROLADOR a implementar */

  /* Remake Kinematic Equations */
  CalcPositionAndOrientation(encoder);
  /* DEBUG */
  printf("REAL: %2f,%2f,%2f  -- ODOM: %2f,%2f,%2f -- ENC: %2f,%2f \n", (m_pcEpuck->GetPosition()).x, (m_pcEpuck->GetPosition()).y, compass[0], m_vPosition.x,m_vPosition.y,m_fOrientation,encoder[0], encoder[1]);
  printf("State: %d\n", m_nState);
  /* DEBUG */
 
  /* Finete States Machine */
  if (m_nState >= m_nPathPlanningStops)
    Stop();
  else if (GoGoal(m_vPositionsPlanning[m_nState].x, m_vPositionsPlanning[m_nState].y, prox))
    m_nState++;


  /* Signal end with leds */
  if (ground[1] == 0.0)
    m_pcEpuck->SetAllColoredLeds( LED_COLOR_GREEN);
  else
    m_pcEpuck->SetAllColoredLeds( LED_COLOR_BLACK);

}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::CalcPositionAndOrientation (double *f_encoder)
{
  /* Remake kinematic equations */
  double fIncU = (f_encoder[0]+ f_encoder[1] )/ 2;
  double fIncTetha = (f_encoder[1] - f_encoder[0])/ CEpuck::WHEELS_DISTANCE;

  /* Substitute arc by chord, take care of 0 division */
  if (fIncTetha != 0.0)
    fIncU = ((f_encoder[0]/fIncTetha)+(CEpuck::WHEELS_DISTANCE/2))* 2.0 * sin (fIncTetha/2.0);

  /* Update new Position */
  m_vPosition.x += fIncU * cos(m_fOrientation + fIncTetha/2);
  m_vPosition.y += fIncU * sin(m_fOrientation + fIncTetha/2);
  
  /* Update new Orientation */
  m_fOrientation += fIncTetha;

  /* Normalize Orientation */
  while(m_fOrientation < 0) m_fOrientation += 2*M_PI;
  while(m_fOrientation > 2*M_PI) m_fOrientation -= 2*M_PI;
}

/******************************************************************************/
/******************************************************************************/

int CIri1Controller::GoGoal (double f_x, double f_y, double *prox)
{
  double fX = (f_x - m_vPosition.x);
  double fY = (f_y - m_vPosition.y);
	double fGoalDirection = 0;
	
  /* If on Goal, return 1 */
  if ( ( fabs(fX) <= ERROR_POSITION ) && ( fabs(fY) <= ERROR_POSITION ) )
    return 1; 
  /* else, calc direction */
	else 
    fGoalDirection = atan2(fY, fX);

  /* Translate fGoalDirection into local coordinates */
  fGoalDirection -= m_fOrientation;
  /* Normalize Direction */
	while ( fGoalDirection > M_PI) fGoalDirection -= 2 * M_PI;
	while ( fGoalDirection < -M_PI) fGoalDirection+=2*M_PI;

  /* Check Prox sensors */
	double fMaxProx = 0.0;
	const double* proxDirections = m_seProx->GetSensorDirections();

	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum of proximity sensors */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
    if (i == 0 || i == 1 || i == 7 || i ==6)
      //if (i != 3 && i != 4 )
    {
      vRepelent.x += prox[i] * cos ( proxDirections[i] );
      vRepelent.y += prox[i] * sin ( proxDirections[i] );
      
      if ( prox[i] > fMaxProx )
        fMaxProx = prox[i];
    }
	}

	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;
	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

  //printf("GOAL: %2f, REPELENT: %2f, ORIEN: %2f\n", fGoalDirection, fRepelent, m_fOrientation);
  /* If obstacle, wight repelent and goal directions */
  if (fMaxProx >= 0.7)
    fGoalDirection = (0.5*fGoalDirection+0.5*fRepelent); 
 
  /*Calc Error direction */
  double fErrorDirection = fGoalDirection; //- m_fOrientation;

	while(fErrorDirection > M_PI) fErrorDirection -= 2*M_PI;
	while(fErrorDirection < -M_PI) fErrorDirection += 2*M_PI;
  /* DEBUG */
  printf("GOAL: %2f, REPELENT: %2f, ORIEN: %2f ERROR: %2f\n", fGoalDirection, fRepelent, m_fOrientation, fErrorDirection);
  /* END DEBUG */

  /* Check ErrorDirection to move right or left */
  if (fErrorDirection > 0)
    TurnAngle(SPEED*(1 - fmin(fErrorDirection, ERROR_DIRECTION)/ERROR_DIRECTION), SPEED);
  else
    TurnAngle(SPEED,SPEED*(1 - fmin(-fErrorDirection, ERROR_DIRECTION)/ERROR_DIRECTION));

  return 0;
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::TurnLeft( double f_custom_speed )
{
	m_acWheels->SetSpeed(-f_custom_speed, f_custom_speed);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::TurnRight( double f_custom_speed )
{
	m_acWheels->SetSpeed(f_custom_speed, -f_custom_speed);
}

/******************************************************************************/
/******************************************************************************/
void CIri1Controller::TurnAngle( double f_custom_speed_left, double f_custom_speed_right  )
{
	m_acWheels->SetSpeed(f_custom_speed_left, f_custom_speed_right);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoForwards( double f_custom_speed )
{
	m_acWheels->SetSpeed(f_custom_speed, f_custom_speed);
}

/******************************************************************************/
/******************************************************************************/


void CIri1Controller::GoBackwards( double f_custom_speed )
{
	m_acWheels->SetSpeed(-f_custom_speed, -f_custom_speed);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Stop( void )
{
	m_acWheels->SetSpeed(0, 0);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::PathPlanning ( void )
{
  /* Create Obstacle Map */
  for ( int y = 0 ; y < m ; y++ )
  {
    for ( int x = 0 ; x < n ; x++ )
    {
      if (mapDraw[y*m+x] == '%' )
        map[x][m-y-1]=1;
    }
  }
  
  /* Obtain start and end desired position */
  int xA=robotStartGridX;
  int yA=robotStartGridY;
  int xB=robotEndGridX;
  int yB=robotEndGridY;

  /* Obtain optimal path */
  string route=pathFind(xA, yA, xB, yB);
  if(route=="") cout<<"An empty route generated!"<<endl;
  cout << "Route:" << route << endl;
  printf("route Length: %d\n", route.length());

  /* Obtain number of changing directions */
  for (int i = 1 ; i < route.length() ; i++)
  {
    //printf("i-1: %c, i: %c\n", route[i-1], route[i]);

    if (route[i-1] != route[i])
      m_nPathPlanningStops++;
  }
  m_nPathPlanningStops++;
  printf("STOPS: %d\n", m_nPathPlanningStops);

  /* Define vector of desired positions. One for each changing direction */
  m_vPositionsPlanning = new dVector2[m_nPathPlanningStops]; 

  /* Calc increment of position, correlating grid and metrics */
  double fXmov = mapLengthX/mapGridX;
  double fYmov = mapLengthY/mapGridY;

  /* Get actual position */
  dVector2 actualPos;
  actualPos.x = robotStartGridX * fXmov;
  actualPos.y = robotStartGridY * fYmov;

  /* Fill vector of desired positions */
  int stop = 0;
  int counter = 0;
  /* Check the route and obtain the positions*/
  for (int i = 1 ; i < route.length() ; i++)
  {
    /* For every position in route, increment countr */
    counter++;
    /* If a direction changed */
    if ((route[i-1] != route[i])) 
    {
      /* Obtain the direction char */
      char c;
      c = route.at(i-1);

      /* Calc the new stop according to actual position and increment based on the grid */
      m_vPositionsPlanning[stop].x = actualPos.x + counter * fXmov*dx[atoi(&c)];
      m_vPositionsPlanning[stop].y = actualPos.y + counter * fYmov*dy[atoi(&c)];

      /* Update position for next stop */
      actualPos.x = m_vPositionsPlanning[stop].x;
      actualPos.y = m_vPositionsPlanning[stop].y;

      /* Increment stop */
      stop++;
      /* reset counter */
      counter = 0;
    }

    /* If we are in the last update, calc last movement */
    if (i==(route.length()-1))
    {
      /* Increment counter */
      counter++;
      /* Obtain the direction char */
      char c;
      c = route.at(i);

      /* DEBUG */
      //printf("COUNTER: %d, CHAR: %c\n", counter, c);
      /* END DEBUG */

      /* Calc the new stop according to actual position and increment based on the grid */
      m_vPositionsPlanning[stop].x = actualPos.x + counter * fXmov*dx[atoi(&c)];// - robotStartGridX * fXmov;
      m_vPositionsPlanning[stop].y = actualPos.y + counter * fYmov*dy[atoi(&c)];// - robotStartGridY * fYmov;

      /* Update position for next stop */
      actualPos.x = m_vPositionsPlanning[stop].x;
      actualPos.y = m_vPositionsPlanning[stop].y;

      /* Increment stop */
      stop++;
      /* reset counter */
      counter = 0;
    }

  }

  /* DEBUG */
  if(route.length()>0)
  {
    int j; char c;
    int x=xA;
    int y=yA;
    map[x][y]=2;
    for ( int i = 0 ; i < route.length() ; i++ )
    {
      c = route.at(i);
      j = atoi(&c); 
      x = x+dx[j];
      y = y+dy[j];
      map[x][y] = 3;
    }
    map[x][y]=4;

    // display the map with the route
    for ( int y = 0 ; y < m ; y++ )
    {
      for ( int x = 0 ; x < n ; x++ )
        if ( map[x][y] == 0 )
          cout<<".";
        else if(map[x][y]==1)
          cout<<"O"; //obstacle
        else if(map[x][y]==2)
          cout<<"S"; //start
        else if(map[x][y]==3)
          cout<<"R"; //route
        else if(map[x][y]==4)
          cout<<"F"; //finish
      cout<<endl;
    }
  }
  /* END DEBUG */

  /* DEBUG */
  //printf("Start: %2f, %2f\n", robotStartGridX * fXmov, robotStartGridY * fXmov);
  //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
    //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
  /* END DEBUG */

  /* Convert to simulator coordinates */
  for (int i = 0 ; i < m_nPathPlanningStops ; i++)
  {
    m_vPositionsPlanning[i].x -= (mapGridX * fXmov)/2;
    m_vPositionsPlanning[i].y -= (mapGridY * fYmov)/2;
    m_vPositionsPlanning[i].y = - m_vPositionsPlanning[i].y;
  }
  /* DEBUG */
  //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
  //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
  /* END DEBUG */


  /* Convert to robot coordinates. FAKE!! */
  for (int i = 0 ; i < m_nPathPlanningStops ; i++)
  {
    /* Traslation */ 
    m_vPositionsPlanning[i].x -= ( (robotStartGridX * fXmov) - (mapGridX * fXmov)/2 );
    m_vPositionsPlanning[i].y += ( (robotStartGridY * fXmov) - (mapGridY * fYmov)/2);
    /* Rotation */
    double compass = m_pcEpuck->GetRotation();
    m_vPositionsPlanning[i].x = m_vPositionsPlanning[i].x * cos (compass) - m_vPositionsPlanning[i].y  * sin(compass);
    m_vPositionsPlanning[i].y = m_vPositionsPlanning[i].x * sin (compass) + m_vPositionsPlanning[i].y  * cos(compass);
  }
  /* DEBUG */
  for (int i = 0 ; i < m_nPathPlanningStops ; i++)
    printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
  /* END DEBUG */
}

/******************************************************************************/
/******************************************************************************/

// A-star algorithm.
// The route returned is a string of direction digits.
string CIri1Controller::pathFind( const int & xStart, const int & yStart, 
    const int & xFinish, const int & yFinish )
{
  static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
  static int pqi; // pq index
  static node* n0;
  static node* m0;
  static int i, j, x, y, xdx, ydy;
  static char c;
  pqi=0;

  // reset the node maps
  for ( y=0 ; y < m ; y++ )
  {
    for ( x = 0 ; x < n ; x++ )
    {
      closed_nodes_map[x][y]=0;
      open_nodes_map[x][y]=0;
    }
  }

  // create the start node and push into list of open nodes
  n0=new node(xStart, yStart, 0, 0);
  n0->updatePriority(xFinish, yFinish);
  pq[pqi].push(*n0);
  open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

  // A* search
  while(!pq[pqi].empty())
  {
    // get the current node w/ the highest priority
    // from the list of open nodes
    n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), 
        pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

    x=n0->getxPos(); y=n0->getyPos();

    pq[pqi].pop(); // remove the node from the open list
    open_nodes_map[x][y]=0;
    // mark it on the closed nodes map
    closed_nodes_map[x][y]=1;

    // quit searching when the goal state is reached
    //if((*n0).estimate(xFinish, yFinish) == 0)
    if(x==xFinish && y==yFinish) 
    {
      // generate the path from finish to start
      // by following the directions
      string path="";
      while(!(x==xStart && y==yStart))
      {
        j=dir_map[x][y];
        c='0'+(j+dir/2)%dir;
        path=c+path;
        x+=dx[j];
        y+=dy[j];
      }

      // garbage collection
      delete n0;
      // empty the leftover nodes
      while(!pq[pqi].empty()) pq[pqi].pop();           
      return path;
    }

    // generate moves (child nodes) in all possible directions
    for ( i = 0 ; i < dir ; i++ )
    {
      xdx=x+dx[i]; ydy=y+dy[i];

      if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || map[xdx][ydy]==1 
            || closed_nodes_map[xdx][ydy]==1))
      {
        // generate a child node
        m0=new node( xdx, ydy, n0->getLevel(), 
            n0->getPriority());
        m0->nextLevel(i);
        m0->updatePriority(xFinish, yFinish);

        // if it is not in the open list then add into that
        if(open_nodes_map[xdx][ydy]==0)
        {
          open_nodes_map[xdx][ydy]=m0->getPriority();
          pq[pqi].push(*m0);
          // mark its parent node direction
          dir_map[xdx][ydy]=(i+dir/2)%dir;
        }
        else if(open_nodes_map[xdx][ydy]>m0->getPriority())
        {
          // update the priority info
          open_nodes_map[xdx][ydy]=m0->getPriority();
          // update the parent direction info
          dir_map[xdx][ydy]=(i+dir/2)%dir;

          // replace the node
          // by emptying one pq to the other one
          // except the node to be replaced will be ignored
          // and the new node will be pushed in instead
          while(!(pq[pqi].top().getxPos()==xdx && 
                pq[pqi].top().getyPos()==ydy))
          {                
            pq[1-pqi].push(pq[pqi].top());
            pq[pqi].pop();       
          }
          pq[pqi].pop(); // remove the wanted node

          // empty the larger size pq to the smaller one
          if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
          while(!pq[pqi].empty())
          {                
            pq[1-pqi].push(pq[pqi].top());
            pq[pqi].pop();       
          }
          pqi=1-pqi;
          pq[pqi].push(*m0); // add the better node instead
        }
        else delete m0; // garbage collection
      }
    }
    delete n0; // garbage collection
  }
  return ""; // no route found
}
