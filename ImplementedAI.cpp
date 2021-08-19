#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>

using namespace std;

/**
* Utility functions & classes
**/

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

        float speed = 0.0f;

        if (isInitialized)
        {
            float xx = (x - lastX) * (x - lastX);
            float yy = (y - lastY) * (y - lastY);
            speed = sqrt(xx + yy) / TimeManager::getDeltaTime();
        }
        else
        {   
            isInitialized = true;         
        }

        cerr << "speed: " << speed << endl;

        lastX = x;
        lastY = y;

        float dirFactor = clamp(abs(nextCheckpointAngle) / 90.0f, 0.0f, 1.0f);
        float distFactor = clamp(nextCheckpointDist / max(speed * 0.1f, 0.01f), 0.0f, 1.0f);
        string strControl;

        // Use boost when the count is at least 1, the orientation is facing the target, and distance is far enough
        if (boostCount > 0 && MathHelper::isApproximatelyZero(dirFactor) && MathHelper::isApproximatelyOne(distFactor) && speed > 8000.0f)
        {
            --boostCount;   
            strControl = "BOOST";
        }
        else 
        {        
            int thrust = (int)(80.0f * distFactor) + (int)(20.0f * clamp(1.1f - dirFactor, 0.0f, 1.0f));
            strControl = to_string(thrust);
        }

        // You have to output the target position
        // followed by the power (0 <= thrust <= 100)
        // i.e.: "x y thrust"
        cout << nextCheckpointX << " " << nextCheckpointY << " " << strControl << endl;
    }
}