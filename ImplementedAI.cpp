#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>

#define MATH_PI 3.14159265f
#define CAR_COUNT_PER_TEAM 2

using namespace std;

/**
* Utility functions & classes
**/

struct Vector2;
struct MapPoint;

class MathHelper
{
private:
    MathHelper() {}

public:
    // this function assumes t is always greater than or equal to zero
    static bool isApproximatelyZero(float t)
    {
        return t < 0.001f;
    }

    // this function assumes t is always greater than or equal to zero
    static bool isApproximatelyOne(float t)
    {
        return t > 0.999f;
    }

    static float rad2deg(float rad)
    {
        return rad / MATH_PI * 180.0f;
    }

    static float deg2rad(float degree)
    {
        return degree / 180.0f * MATH_PI;        
    }
};

struct Vector2
{
    float m_X;
    float m_Y;

    Vector2 operator+(const Vector2& rhs) const
    {
        return Vector2 { m_X + rhs.m_X, m_Y + rhs.m_Y };
    }

    Vector2 operator-(const Vector2& rhs) const
    {
        return Vector2 { m_X - rhs.m_X, m_Y - rhs.m_Y };
    }    

    Vector2 operator*(const float multiplier) const
    {
        return Vector2 { m_X * multiplier, m_Y * multiplier };
    }

    Vector2 operator/(const float divider) const
    {
        return Vector2 { m_X / divider, m_Y / divider };
    }

    float getMagnitude()
    {
        return sqrt(m_X * m_X + m_Y * m_Y);
    }

    Vector2 getNormalized()
    {
        return (this->getMagnitude() > 0.001f) ? (*this) / this->getMagnitude() : (*this);
    }       

    float getTangentAngle()
    {
        return MathHelper::rad2deg(atan2(m_Y, m_X));
    } 

    static float dotProduct(const Vector2& lhs, const Vector2& rhs)
    {
        return (lhs.m_X * rhs.m_X + lhs.m_Y * rhs.m_Y);
    }   
};

struct MapPoint
{
    int m_X;
    int m_Y;    

    Vector2 toVector()
    {
        return Vector2 { (float)m_X, (float)m_Y };
    }  

    bool operator==(const MapPoint& rhs) const
    {
        return m_X == rhs.m_X && m_Y == rhs.m_Y;
    }

    bool operator!=(const MapPoint& rhs) const
    {
        return m_X != rhs.m_X || m_Y != rhs.m_Y;
    }    

    MapPoint operator+(const MapPoint& rhs) const
    {
        return MapPoint { m_X + rhs.m_X, m_Y + rhs.m_Y };
    }

    MapPoint operator-(const MapPoint& rhs) const
    {
        return MapPoint { m_X - rhs.m_X, m_Y - rhs.m_Y };
    }    
};

class TimeManager
{
private:
    TimeManager() {};

public:
    static void initialize()
    {        
        s_LastCheckedTime = chrono::steady_clock::now(); 
    }

    static void update()
    {        
        auto currentTime = chrono::steady_clock::now();
        s_DeltaTime = chrono::duration_cast<chrono::milliseconds>(currentTime - s_LastCheckedTime).count() / 1000.0f;
    }

    static float getDeltaTime()
    {
        return s_DeltaTime;
    }

private:    
    static chrono::time_point<chrono::steady_clock> s_LastCheckedTime;
    static float s_DeltaTime;
};

chrono::time_point<chrono::steady_clock> TimeManager::s_LastCheckedTime;
float TimeManager::s_DeltaTime;

struct Circuit
{    
    vector<MapPoint> m_Checkpoints;

    bool isPotentiallyIdentical(const Circuit& rhs) const
    {
        if (m_Checkpoints.empty() || rhs.m_Checkpoints.empty())
        {
            return false;
        }

        int size = min(m_Checkpoints.size(), rhs.m_Checkpoints.size());

        for (int i = 0; i < size; ++i)
        {
            auto diff = (m_Checkpoints[i] - rhs.m_Checkpoints[i]).toVector();

            if (diff.getMagnitude() > 50.0f)
            {
                return false;
            }
        }

        return true;
    }

    void copyFrom(const Circuit& target)
    {
        int start = m_Checkpoints.size();
        int size = target.m_Checkpoints.size();
        
        for (int i = start; i < size; ++i)
        {
            m_Checkpoints.push_back(target.m_Checkpoints[i]);
        }
    }
};

class CircuitManager
{
private:    
    CircuitManager() {};

public:
    static void initialize()
    {
    }

    static void setMaxLapNumber(int maxLabNumber)
    {
        s_MaxLabNumber = maxLabNumber;
    }

    static void registerCircuit(const Circuit& circuit)
    {
        s_CurrentCircuit = circuit;
        s_IsAnalyzed = true;
    }

    static MapPoint getCheckpoint(int checkpointIndex)
    {
        int safeIndex = (checkpointIndex + s_CurrentCircuit.m_Checkpoints.size()) % s_CurrentCircuit.m_Checkpoints.size();
        return s_CurrentCircuit.m_Checkpoints[safeIndex];
    }

    static bool hasAnalyzeDone()
    {   
        return s_IsAnalyzed;     
    }

    static bool isLastCheckpoint(int checkpointIndex)
    {
        return (checkpointIndex == s_CurrentCircuit.m_Checkpoints.size() - 1);
    }

    static bool isLastLap(int lapNumber)
    {        
        return lapNumber >= s_MaxLabNumber;
    }

private:
    static Circuit s_CurrentCircuit;
    static bool s_IsAnalyzed;
    static int s_MaxLabNumber;
};

Circuit CircuitManager::s_CurrentCircuit;
bool CircuitManager::s_IsAnalyzed ;
int CircuitManager::s_MaxLabNumber;

class CarState
{
public:
    CarState(string& name, bool isControllable)
    {        
        m_Name = name;
        m_BoostCount = 1;
        m_IsControllable = isControllable;
        m_CurrentLapNumber = 1;
        m_CurrentCheckpointIndex = -1;
        m_Dir = Vector2{ 0.0f, 0.0f };
    }

public:
    string& getName()
    {
        return m_Name;
    }

    void update(int x, int y, int vx, int vy, int facingAngle, int checkpointIndex)
    {     
        m_Pos = MapPoint { x, y };

        auto prevDir = m_Dir;
        m_Dir = MapPoint { vx, vy }.toVector();
        m_Speed = m_Dir.getMagnitude();
        m_Dir = m_Dir.getNormalized();
        m_AngularSpeed = (m_Dir - prevDir).getTangentAngle() / TimeManager::getDeltaTime();
        m_MaxSpeed = max(m_MaxSpeed, m_Speed);
        m_MaxAngularSpeed = max(m_MaxAngularSpeed, abs(m_AngularSpeed));
        m_FancingAngle = facingAngle;

        if (m_CurrentCheckpointIndex != checkpointIndex)
        {   
            if (checkpointIndex == 0)         
            {                
                ++m_CurrentLapNumber;
            }
            
            m_CurrentCheckpointIndex = checkpointIndex;
        }
    }

    float getSpeed()
    {
        return m_Speed;
    }

    float getAngularSpeed()
    {
        return m_AngularSpeed;
    }

    float getMaxSpeed()
    {
        return m_MaxSpeed;
    }

    float getMaxAngularSpeed()
    {
        return m_MaxAngularSpeed;
    }

    int getCurrentLapNumber()
    {
        return m_CurrentLapNumber;
    }

    int getCurrentCheckpointIndex()
    {
        return m_CurrentCheckpointIndex;
    }

    void makeDesicion(int& outTargetX, int& outTargetY, string& outTargetAction)
    {        
        float minThrust = 10.0f;
        float maxThrust = 100.0f;
        float distFactor = 0.0f;
        bool isNext = false;

        auto currentCheckpoint = CircuitManager::getCheckpoint(m_CurrentCheckpointIndex);
        auto toCurrent = (currentCheckpoint - m_Pos).toVector();
        float distanceToNextCheckpoint = toCurrent.getMagnitude();

        toCurrent = toCurrent.getNormalized();
        float angleToNextCheckpoint = toCurrent.getTangentAngle() - m_FancingAngle;

        if (m_IsControllable)
        {
            outTargetX = currentCheckpoint.m_X;
            outTargetY = currentCheckpoint.m_Y;
            distFactor = max(distanceToNextCheckpoint * cos(MathHelper::deg2rad(angleToNextCheckpoint)), 0.0f);
            distFactor = (m_Speed > 0.001f) ? clamp(distFactor / (m_Speed * 10.0f), 0.0f, 1.0f) : 1.0f;

            if (CircuitManager::hasAnalyzeDone() && !CircuitManager::isLastLap(m_CurrentLapNumber) && !CircuitManager::isLastCheckpoint(m_CurrentCheckpointIndex))
            {   
                auto nextCheckpoint = CircuitManager::getCheckpoint(m_CurrentCheckpointIndex + 1);
                float movementCosine = max(Vector2::dotProduct(m_Dir, toCurrent), 0.0f);

                if (movementCosine > cos(MathHelper::deg2rad(30.0f)) && distanceToNextCheckpoint < (m_Speed * 10.0f))
                {                    
                    isNext = true;
                    currentCheckpoint = CircuitManager::getCheckpoint(m_CurrentCheckpointIndex + 1);
                    nextCheckpoint = CircuitManager::getCheckpoint(m_CurrentCheckpointIndex + 2);
                    toCurrent = (currentCheckpoint - m_Pos).toVector().getNormalized();

                    auto diff = (currentCheckpoint - m_Pos).toVector();
                    distanceToNextCheckpoint = diff.getMagnitude();
                    distFactor = max(distanceToNextCheckpoint * Vector2::dotProduct(m_Dir, toCurrent), 0.0f);
                    distFactor = (m_Speed > 0.001f) ? clamp(distFactor / (m_Speed * 10.0f), 0.0f, 1.0f) : 1.0f;                    
                }

                auto halfVector = toCurrent + m_Dir;
                halfVector = halfVector.getNormalized();
                Vector2 toNext = (nextCheckpoint - currentCheckpoint).toVector().getNormalized();
                minThrust = max(Vector2::dotProduct(halfVector, toNext) * maxThrust, 0.0f);
            }            

            // Use boost in the last lap
            if (m_BoostCount > 0 && CircuitManager::isLastLap(m_CurrentLapNumber) && distFactor > 0.8f && abs(angleToNextCheckpoint) < 10)
            {
                cerr << "BOOST!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                --m_BoostCount;   
                outTargetAction = "BOOST";
            }
            else 
            {        
                int thrust = minThrust + (maxThrust - minThrust) * distFactor;
                outTargetAction = to_string(thrust);
            }    

            // Calibrate target position
            auto adjustment = (toCurrent * Vector2::dotProduct(m_Dir, toCurrent)) - m_Dir;
            adjustment = adjustment * m_Speed * 10.0f;
            outTargetX = currentCheckpoint.m_X + adjustment.m_X;
            outTargetY = currentCheckpoint.m_Y + adjustment.m_Y;             
        }

        cerr << endl;
        cerr << "[" << m_Name << "]" << endl;
        cerr << "inputs: " << distanceToNextCheckpoint << ", " << angleToNextCheckpoint << endl;
        cerr << "dir: " << m_Dir.m_X << ", " << m_Dir.m_Y << endl;
        cerr << "boostCnt: " << m_BoostCount << endl;
        cerr << "speed: (" << m_Speed << " / " << m_MaxSpeed << ")" << endl;
        cerr << "angularSpeed: (" << m_AngularSpeed << " / " << m_MaxAngularSpeed << ")" << endl;
        cerr << "factors: " << distFactor << endl;
        cerr << "isNext: " << isNext << endl;
        cerr << "thrust range: " << minThrust << " ~ " << maxThrust << endl;        
    }

private:
    int m_BoostCount;
    MapPoint m_Pos;
    Vector2 m_Dir;
    float m_Speed;
    float m_AngularSpeed;
    float m_MaxSpeed;
    float m_MaxAngularSpeed;
    string m_Name;
    bool m_IsControllable;
    int m_CurrentLapNumber;
    int m_CurrentCheckpointIndex;
    float m_FancingAngle;
};

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/

int main()
{
    int boostCount = 1;
    bool isInitialized = false;
    int lastX, lastY;
    TimeManager::initialize();
    CircuitManager::initialize();
    CarState* myCarStatePtrs[CAR_COUNT_PER_TEAM];
    CarState* bossCarStatePtrs[CAR_COUNT_PER_TEAM];

    for (int i = 0; i < CAR_COUNT_PER_TEAM; ++i)
    {
        string myName = "MyCar " + to_string(i);
        myCarStatePtrs[i] = new CarState(myName, true);

        string bossName = "Boss " + to_string(i);
        bossCarStatePtrs[i] = new CarState(bossName, false);
    }

    // initialization
    {
        int laps;
        int checkpointCount;
        cin >> laps; cin.ignore();
        cin >> checkpointCount; cin.ignore();

        cerr << "[Intiailization]" << endl;
        cerr << "laps: " << laps << endl;
        cerr << "checkpointCount: " << checkpointCount << endl;

        Circuit circuit;
        circuit.m_Checkpoints.reserve(checkpointCount);

        for (int i = 0; i < checkpointCount; ++i)
        {
            int checkpointX, checkpointY;
            cin >> checkpointX >> checkpointY; cin.ignore();
            circuit.m_Checkpoints.push_back(MapPoint { checkpointX, checkpointY });

            cerr << "\t" << i << " - " << checkpointX << ", " << checkpointY << endl;
        }

        CircuitManager::setMaxLapNumber(laps);
        CircuitManager::registerCircuit(circuit);
    }

    // game loop
    while (1) 
    {
        TimeManager::update();

        // inputs for my team
        cerr << endl;
        cerr << "[inputs for my team]" << endl;

        for (int i = 0; i < CAR_COUNT_PER_TEAM; ++i)
        {          
            int x, y;
            int vx, vy;
            int angle;
            int nextCheckpointID;
            cin >> x >> y >> vx >> vy >> angle >> nextCheckpointID;
            myCarStatePtrs[i]->update(x, y, vx, vy, angle, nextCheckpointID);

            cerr << "\t" << i << ") " << x << ", " << y << ", " << vx << ", " << vy << ", " << angle << ", " << nextCheckpointID << endl;
        }        

        // inputs for boss team
        cerr << endl;
        cerr << "[inputs for boss team]" << endl;

        for (int i = 0; i < CAR_COUNT_PER_TEAM; ++i)
        {          
            int x, y;
            int vx, vy;
            int angle;
            int nextCheckpointID;
            cin >> x >> y >> vx >> vy >> angle >> nextCheckpointID;
            bossCarStatePtrs[i]->update(x, y, vx, vy, angle, nextCheckpointID);

            cerr << "\t" << i << ") " << x << ", " << y << ", " << vx << ", " << vy << ", " << angle << ", " << nextCheckpointID << endl;
        }                

        // outputs
        for (int i = 0; i < CAR_COUNT_PER_TEAM; ++i)
        {            
            int targetX, targetY;
            string targetAction;
            myCarStatePtrs[i]->makeDesicion(targetX, targetY, targetAction);
            cout << targetX << " " << targetY << " " << targetAction << endl;
        }

        // this is just for debugging purpose
        for (int i = 0; i < CAR_COUNT_PER_TEAM; ++i)
        {            
            int targetX, targetY;
            string targetAction;
            bossCarStatePtrs[i]->makeDesicion(targetX, targetY, targetAction);            
        }
    }

    for (int i = 0; i < CAR_COUNT_PER_TEAM; ++i)
    {
        delete myCarStatePtrs[i];
        delete bossCarStatePtrs[i];
    }    
}