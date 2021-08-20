#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>

#define MAX_LAP_NUMBER 3

using namespace std;

/**
* Utility functions & classes
**/

struct Vector2
{
    float m_X;
    float m_Y;
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

    static Vector2 divide(Vector2 vec, float divider)
    {
        return Vector2 {vec.m_X / divider, vec.m_Y / divider };
    }

    static Vector2 getNormalizedVector(int x, int y)
    {
        auto vec = Vector2 { (float)x, (float)y };
        return divide(vec, getMagnitude(vec));
    }

    static float dotProduct(Vector2 vec1, Vector2 vec2)
    {
        return (vec1.m_X * vec2.m_X + vec1.m_Y * vec2.m_Y);
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

struct MapPoint
{
    int m_X;
    int m_Y;    

    bool operator==(const MapPoint& rhs) const
    {
        return m_X == rhs.m_X && m_Y == rhs.m_Y;
    }
};

class CarState
{
public:
    CarState()
    {        
        m_BoostCount = 1;
    }

public:
    void setPosition(int x, int y)
    {   
        if (m_HasPosSet)    
        {   
            m_Dir = Vector2 { (float)(x - m_Pos.m_X), (float)(y - m_Pos.m_Y) };
            m_Speed = MathHelper::getMagnitude(m_Dir) / TimeManager::getDeltaTime();
            m_Speed = max(m_Speed, 0.001f);
            m_Dir = MathHelper::divide(m_Dir, m_Speed);
        }
        else
        {
            m_HasPosSet = true;
            m_Dir = Vector2 { 0.0f, 0.0f };
            m_Speed = 0.0f;            
        }

        m_Pos = MapPoint { x, y };
    }

    float getSpeed()
    {
        return m_Speed;
    }

    void makeDecision(int checkPointX, int checkPointY, float distanceToNextCheckpoint, float angleToNextCheckpoint, int& outTargetX, int& outTargetY, string& outTargetAction)
    {
        outTargetX = checkPointX;
        outTargetY = checkPointY;

        float dirFactor = clamp(abs(angleToNextCheckpoint) / 90.0f, 0.0f, 1.0f);
        float distFactor = clamp(distanceToNextCheckpoint / max(m_Speed * 0.2f, 0.01f), 0.0f, 1.0f);

        // Use boost when the count is at least 1, the orientation is facing the target, and distance is far enough
        if (m_BoostCount > 0 && MathHelper::isApproximatelyZero(dirFactor) && MathHelper::isApproximatelyOne(distFactor) && m_Speed > 8000.0f)
        {
            --m_BoostCount;   
            outTargetAction = "BOOST";
        }
        else 
        {        
            int thrust = (int)(70.0f * distFactor) + (int)(30.0f * clamp(1.1f - dirFactor, 0.0f, 1.0f));
            outTargetAction = to_string(thrust);
        }                
    }

private:
    bool m_HasPosSet = false;
    int m_BoostCount;
    MapPoint m_Pos;
    Vector2 m_Dir;
    float m_Speed;
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
    CarState myCarState;

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
        myCarState.setPosition(x, y);

        cerr << "speed: " << myCarState.getSpeed() << endl;

        int targetX, targetY;
        string targetAction;
        myCarState.makeDecision(nextCheckpointX, nextCheckpointY, nextCheckpointDist, nextCheckpointAngle, targetX, targetY, targetAction);

        // You have to output the target position
        // followed by the power (0 <= thrust <= 100)
        // i.e.: "x y thrust"
        cout << targetX << " " << targetY << " " << targetAction << endl;
    }
}