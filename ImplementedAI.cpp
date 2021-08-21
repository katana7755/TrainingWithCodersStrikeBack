#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>

#define MAX_LAP_NUMBER 3
#define MATH_PI 3.14159265f

using namespace std;

/**
* Utility functions & classes
**/

struct Vector2
{
    float m_X;
    float m_Y;
};

struct MapPoint
{
    int m_X;
    int m_Y;    

    bool operator==(const MapPoint& rhs) const
    {
        return m_X == rhs.m_X && m_Y == rhs.m_Y;
    }

    bool operator!=(const MapPoint& rhs) const
    {
        return m_X != rhs.m_X || m_Y != rhs.m_Y;
    }    
};

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

    static float getMagnitude(Vector2 vec)
    {
        return sqrt(vec.m_X * vec.m_X + vec.m_Y * vec.m_Y);
    }

    static Vector2 plus(Vector2 vec1, Vector2 vec2)
    {        
        return Vector2 { vec1.m_X + vec2.m_X, vec1.m_Y + vec2.m_Y };
    }

    static Vector2 minus(Vector2 vec1, Vector2 vec2)
    {        
        return Vector2 { vec1.m_X - vec2.m_X, vec1.m_Y - vec2.m_Y };
    }

    static Vector2 multiply(Vector2 vec, float multiply)
    {
        return Vector2 {vec.m_X * multiply, vec.m_Y * multiply };
    }

    static Vector2 divide(Vector2 vec, float divider)
    {
        return Vector2 {vec.m_X / divider, vec.m_Y / divider };
    }

    static Vector2 getNormalizedVector(int x, int y)
    {
        auto vec = Vector2 { (float)x, (float)y };
        return divide(vec, getMagnitude(vec));
    }

    static Vector2 getNormalizedVector(Vector2 vec)
    {
        return divide(vec, getMagnitude(vec));
    }    

    static float dotProduct(Vector2 vec1, Vector2 vec2)
    {
        return (vec1.m_X * vec2.m_X + vec1.m_Y * vec2.m_Y);
    }

    static float getAngle(Vector2 vec)
    {
        return atan2(vec.m_Y, vec.m_X) * 180.0f / 3.14159265;
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
            auto diff = Vector2 { (float)(m_Checkpoints[i].m_X - rhs.m_Checkpoints[i].m_X), (float)(m_Checkpoints[i].m_Y - rhs.m_Checkpoints[i].m_Y) };

            if (MathHelper::getMagnitude(diff) > 50.0f)
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
        s_CurrentLapNumber = 1;
    }

    static void updateCheckpoint(int x, int y)
    {     
        auto checkPoint = MapPoint { x, y };
        auto it = find(s_CurrentCircuit.m_Checkpoints.begin(), s_CurrentCircuit.m_Checkpoints.end(), checkPoint);

        if (it != s_CurrentCircuit.m_Checkpoints.end())
        {
            int index = distance(s_CurrentCircuit.m_Checkpoints.begin(), it);

            if (index != s_CurrentCheckPointIndex)
            {
                s_IsAnalyzed = true;            
                s_CurrentCheckPointIndex = index;
                s_CurrentLapNumber = (s_CurrentCheckPointIndex == 0) ? s_CurrentLapNumber + 1 : s_CurrentLapNumber;
            }
        }
        else
        {
            s_CurrentCheckPointIndex = s_CurrentCircuit.m_Checkpoints.size();
            s_CurrentCircuit.m_Checkpoints.push_back(checkPoint);

            if (s_PossibleCircuitList.size() > 0)
            {
                auto iter = --s_PossibleCircuitList.end();
                int size = s_PossibleCircuitList.size();

                for (int i = 0; i < size; ++i, --iter)
                {
                    if (!iter->isPotentiallyIdentical(s_CurrentCircuit))
                    {
                        s_PossibleCircuitList.erase(iter);
                    }
                }

                if (s_PossibleCircuitList.size() == 1)
                {
                    s_CurrentCircuit.copyFrom(s_PossibleCircuitList[0]);
                    s_IsAnalyzed = true;
                }
            }
        }

        cerr << "CircuitManager: " << s_CurrentCheckPointIndex << ", " << s_CurrentLapNumber << endl;
    }

    static bool hasAnalyzeDone()
    {   
        return s_IsAnalyzed;     
    }

    static bool isTargettingLastCheckpoint()
    {
        return (s_CurrentLapNumber >= MAX_LAP_NUMBER && s_CurrentCheckPointIndex == s_CurrentCircuit.m_Checkpoints.size() - 1);
    }

    static bool isLastLap()
    {        
        return s_CurrentLapNumber >= MAX_LAP_NUMBER;
    }

    static MapPoint getCheckpointFromCurrent(int step)
    {        
        return s_CurrentCircuit.m_Checkpoints[(s_CurrentCheckPointIndex + step + s_CurrentCircuit.m_Checkpoints.size()) % s_CurrentCircuit.m_Checkpoints.size()];
    }

    static void printCircuit()
    {
        if (s_IsAnalyzed == false)
        {
            return;
        }

        cerr << endl;
        cerr << "[Analyzed Circuit]" << endl;
        cerr << "\tCircuit" << endl;
        cerr << "\t{" << endl;
        cerr << "\t\t{" << endl;

        for (auto point : s_CurrentCircuit.m_Checkpoints)
        {
            cerr << "\t\t\tMapPoint { " << point.m_X << ", " << point.m_Y << " }," <<endl;
        }

        cerr << "\t\t}" << endl;
        cerr << "\t}," << endl;
    }

private:
    static Circuit s_CurrentCircuit;
    static bool s_IsAnalyzed;
    static int s_CurrentLapNumber;
    static int s_CurrentCheckPointIndex;
    static vector<Circuit> s_PossibleCircuitList;
};

Circuit CircuitManager::s_CurrentCircuit;
bool CircuitManager::s_IsAnalyzed;
int CircuitManager::s_CurrentLapNumber;
int CircuitManager::s_CurrentCheckPointIndex;
vector<Circuit> CircuitManager::s_PossibleCircuitList = 
{
    // Circuit 
    // {
    //     {
    //         MapPoint { 7282, 6658 },
    //         MapPoint { 5424, 2828 },
    //         MapPoint { 10338, 3358 },
    //         MapPoint { 11174, 5407 },
    //     }
    // },
    // Circuit 
    // {
    //     {
    //         MapPoint { 10587, 5059 },
    //         MapPoint { 13109, 2300 },
    //         MapPoint { 4570, 2154 },
    //         MapPoint { 7325, 4930 },
    //         MapPoint { 3315, 7214 },
    //         MapPoint { 14563, 7710 },
    //     }
    // },  
    // Circuit 
    // {
    //     {
    //         MapPoint { 9101, 1854 },
    //         MapPoint { 5025, 5238 },
    //         MapPoint { 11459, 6081 },
    //     }
    // },      
	// Circuit
	// {
	// 	{
	// 		MapPoint { 13591, 7574 },
	// 		MapPoint { 12454, 1320 },
	// 		MapPoint { 10535, 5986 },
	// 		MapPoint { 3596, 5175 },
	// 	}
	// },     
    // Circuit
	// {
	// 	{
	// 		MapPoint { 10570, 5960 },
	// 		MapPoint { 3565, 5161 },
	// 		MapPoint { 13563, 7587 },
	// 		MapPoint { 12481, 1323 },
	// 	}
	// },
	// Circuit
	// {
	// 	{
	// 		MapPoint { 13482, 2346 },
	// 		MapPoint { 12922, 7204 },
	// 		MapPoint { 5653, 2562 },
	// 		MapPoint { 4101, 7426 },
	// 	}
	// },    
	// Circuit
	// {
	// 	{
	// 		MapPoint { 9575, 1428 },
	// 		MapPoint { 3635, 4436 },
	// 		MapPoint { 7970, 7901 },
	// 		MapPoint { 13326, 5531 },
	// 	}
	// },    
	// Circuit
	// {
	// 	{
	// 		MapPoint { 7264, 6676 },
	// 		MapPoint { 5451, 2814 },
	// 		MapPoint { 10324, 3351 },
	// 		MapPoint { 11225, 5442 },
	// 	}
	// },   
	// Circuit
	// {
	// 	{
	// 		MapPoint { 14688, 1381 },
	// 		MapPoint { 3426, 7213 },
	// 		MapPoint { 9435, 7216 },
	// 		MapPoint { 5993, 4263 },
	// 	}
	// },     
	// Circuit
	// {
	// 	{
	// 		MapPoint { 8000, 7917 },
	// 		MapPoint { 13281, 5533 },
	// 		MapPoint { 9549, 1398 },
	// 		MapPoint { 3646, 4448 },
	// 	}
	// },  
	// Circuit
	// {
	// 	{
	// 		MapPoint { 3621, 5263 },
	// 		MapPoint { 13855, 5072 },
	// 		MapPoint { 10670, 2299 },
	// 		MapPoint { 8686, 7431 },
	// 		MapPoint { 7188, 2161 },
	// 	}
	// },   
	// Circuit
	// {
	// 	{
	// 		MapPoint { 11195, 5453 },
	// 		MapPoint { 7287, 6654 },
	// 		MapPoint { 5438, 2822 },
	// 		MapPoint { 10320, 3369 },
	// 	}
	// },       
	// Circuit
	// {
	// 	{
	// 		MapPoint { 7642, 5958 },
	// 		MapPoint { 3128, 7536 },
	// 		MapPoint { 9491, 4368 },
	// 		MapPoint { 14533, 7805 },
	// 		MapPoint { 6313, 4262 },
	// 		MapPoint { 7798, 884 },
	// 	}
	// },    
};

class CarState
{
public:
    CarState(const char* name, bool isControllable)
    {        
        m_HasPosSet = false;
        m_Name = name;
        m_BoostCount = 1;
        m_IsControllable = isControllable;
    }

public:
    void setPosition(int x, int y)
    {   
        if (m_HasPosSet)    
        {   
            auto prevDir = m_Dir;
            m_Dir = Vector2 { (float)(x - m_Pos.m_X), (float)(y - m_Pos.m_Y) };
            m_Speed = MathHelper::getMagnitude(m_Dir) / TimeManager::getDeltaTime();
            m_Speed = max(m_Speed, 0.001f);
            m_Dir = MathHelper::getNormalizedVector(m_Dir);
            m_AngularSpeed = MathHelper::getAngle(MathHelper::minus(m_Dir, prevDir)) / TimeManager::getDeltaTime();
            m_MaxSpeed = max(m_MaxSpeed, m_Speed);
            m_MaxAngularSpeed = max(m_MaxAngularSpeed, abs(m_AngularSpeed));
        }
        else
        {
            m_HasPosSet = true;
            m_Dir = Vector2 { 0.0f, 0.0f };
            m_Speed = 0.0f;  
            m_AngularSpeed = 0.0f;          
            m_MaxSpeed = 0.0f;
            m_MaxAngularSpeed = 0.0f;
        }

        m_Pos = MapPoint { x, y };
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

    void makeDecision(int checkPointX, int checkPointY, float distanceToNextCheckpoint, float angleToNextCheckpoint, int& outTargetX, int& outTargetY, string& outTargetAction)
    {
        float minThrust = 10.0f;
        float maxThrust = 100.0f;
        float distFactor = 0.0f;
        bool isNext = false;

        if (m_IsControllable)
        {
            outTargetX = checkPointX;
            outTargetY = checkPointY;
            distFactor = max(distanceToNextCheckpoint * cos(angleToNextCheckpoint / 180.0f * MATH_PI), 0.0f);
            distFactor = (m_Speed > 0.001f) ? clamp(distFactor / (m_Speed * 10.0f), 0.0f, 1.0f) : 1.0f;

            MapPoint currentCheckpoint = MapPoint { checkPointX, checkPointY };
            Vector2 toCurrent = MathHelper::getNormalizedVector(currentCheckpoint.m_X - m_Pos.m_X, currentCheckpoint.m_Y - m_Pos.m_Y);

            if (CircuitManager::hasAnalyzeDone() && !CircuitManager::isTargettingLastCheckpoint())
            {   
                MapPoint nextCheckpoint = MapPoint { checkPointX, checkPointY };
                float movementCosine = max(MathHelper::dotProduct(m_Dir, toCurrent), 0.0f);

                if (movementCosine > cos(MATH_PI * 30.0f / 180.0f) && distanceToNextCheckpoint < (m_Speed * 10.0f))
                {                    
                    isNext = true;
                    currentCheckpoint = CircuitManager::getCheckpointFromCurrent(1);
                    nextCheckpoint = CircuitManager::getCheckpointFromCurrent(2);
                    toCurrent = MathHelper::getNormalizedVector(currentCheckpoint.m_X - m_Pos.m_X, currentCheckpoint.m_Y - m_Pos.m_Y);

                    auto diff = Vector2 { (float)(currentCheckpoint.m_X - m_Pos.m_X), (float)(currentCheckpoint.m_Y - m_Pos.m_Y) };
                    distanceToNextCheckpoint = MathHelper::getMagnitude(diff);
                    distFactor = max(distanceToNextCheckpoint * MathHelper::dotProduct(m_Dir, toCurrent), 0.0f);
                    distFactor = (m_Speed > 0.001f) ? clamp(distFactor / (m_Speed * 10.0f), 0.0f, 1.0f) : 1.0f;                    
                }
                else
                {
                    nextCheckpoint = CircuitManager::getCheckpointFromCurrent(1);
                }

                auto toCurrentHalf = MathHelper::plus(toCurrent, m_Dir);
                toCurrentHalf = MathHelper::getNormalizedVector(toCurrent);
                Vector2 toNext = MathHelper::getNormalizedVector(nextCheckpoint.m_X - currentCheckpoint.m_X, nextCheckpoint.m_Y - currentCheckpoint.m_Y);
                minThrust = max(MathHelper::dotProduct(toCurrentHalf, toNext) * maxThrust, 0.0f);
            }            

            // Use boost in the last lap
            if (m_BoostCount > 0 && CircuitManager::isLastLap() && distFactor > 0.8f && abs(angleToNextCheckpoint) < 10)
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
            auto adjustment = MathHelper::minus(MathHelper::multiply(toCurrent, MathHelper::dotProduct(m_Dir, toCurrent)), m_Dir);
            adjustment = MathHelper::multiply(adjustment, m_Speed * 10.0);
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
    bool m_HasPosSet;
    int m_BoostCount;
    MapPoint m_Pos;
    Vector2 m_Dir;
    float m_Speed;
    float m_AngularSpeed;
    float m_MaxSpeed;
    float m_MaxAngularSpeed;
    string m_Name;
    bool m_IsControllable;
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
    CarState myCarState("MyCar", true);
    CarState bossCarState("Boss", false);

    // game loop
    while (1) {
        int x;
        int y;
        int nextCheckpointX; // x position of the next check point
        int nextCheckpointY; // y position of the next check point
        int nextCheckpointDist; // distance to the next checkpoint
        int nextCheckpointAngle; // angle between your pod orientation and the direction of the next checkpoint
        cin >> x >> y >> nextCheckpointX >> nextCheckpointY >> nextCheckpointDist >> nextCheckpointAngle; cin.ignore();
        int opponentX;
        int opponentY;
        cin >> opponentX >> opponentY; cin.ignore();

        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;
        cerr << "distance to the next target: " << nextCheckpointDist << endl;

        TimeManager::update();
        CircuitManager::updateCheckpoint(nextCheckpointX, nextCheckpointY);
        myCarState.setPosition(x, y);
        bossCarState.setPosition(opponentX, opponentY);

        cerr << "target: " << nextCheckpointX << ", " << nextCheckpointY << " and " << nextCheckpointAngle << endl;
        cerr << "hasCircuitAnalyzed: " << CircuitManager::hasAnalyzeDone() << endl;

        int targetX, targetY;
        string targetAction;
        myCarState.makeDecision(nextCheckpointX, nextCheckpointY, nextCheckpointDist, nextCheckpointAngle, targetX, targetY, targetAction);
        bossCarState.makeDecision(nextCheckpointX, nextCheckpointY, nextCheckpointDist, nextCheckpointAngle, targetX, targetY, targetAction);
        CircuitManager::printCircuit();

        // You have to output the target position
        // followed by the power (0 <= thrust <= 100)
        // i.e.: "x y thrust"
        cout << targetX << " " << targetY << " " << targetAction << endl;
    }
}