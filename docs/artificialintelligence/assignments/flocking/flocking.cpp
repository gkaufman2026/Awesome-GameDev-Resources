#include <cmath>
#include <iomanip>
#include <iostream>
#include <utility>
#include <vector>

using namespace std;

struct Vector2 {
  double x = 0, y = 0;
  Vector2() : x(0), y(0){};
  Vector2(double x, double y) : x(x), y(y){};
  Vector2(const Vector2& v) = default;

  // unary operations
  Vector2 operator-() const { return {-x, -y}; }
  Vector2 operator+() const { return {x, y}; }

  // binary operations
  Vector2 operator-(const Vector2& rhs) const { return {x - rhs.x, y - rhs.y}; }
  Vector2 operator+(const Vector2& rhs) const { return {x + rhs.x, y + rhs.y}; }
  Vector2 operator*(const double& rhs) const { return {x * rhs, y * rhs}; }
  friend Vector2 operator*(const double& lhs, const Vector2& rhs) {
    return {lhs * rhs.x, lhs * rhs.y};
  }
  Vector2 operator/(const double& rhs) const { return {x / rhs, y / rhs}; }
  Vector2 operator/(const Vector2& rhs) const { return {x / rhs.x, y / rhs.y}; }
  bool operator!=(const Vector2& rhs) const { return (*this - rhs).sqrMagnitude() >= 1.0e-6; };
  bool operator==(const Vector2& rhs) const { return (*this - rhs).sqrMagnitude() < 1.0e-6; };

  // assignment operation
  Vector2& operator=(Vector2 const& rhs) = default;
  Vector2& operator=(Vector2&& rhs) = default;

  // compound assignment operations
  Vector2& operator+=(const Vector2& rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }
  Vector2& operator-=(const Vector2& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }
  Vector2& operator*=(const double& rhs) {
    x *= rhs;
    y *= rhs;
    return *this;
  }
  Vector2& operator/=(const double& rhs) {
    x /= rhs;
    y /= rhs;
    return *this;
  }
  Vector2& operator*=(const Vector2& rhs) {
    x *= rhs.x;
    y *= rhs.y;
    return *this;
  }
  Vector2& operator/=(const Vector2& rhs) {
    x /= rhs.x;
    y /= rhs.y;
    return *this;
  }

  double sqrMagnitude() const { return x * x + y * y; }
  double getMagnitude() const { return sqrt(sqrMagnitude()); }
  static double getMagnitude(const Vector2& vector) { return vector.getMagnitude(); }

  static double Distance(const Vector2& a, const Vector2& b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
  };
  double Distance(const Vector2& b) const {
    return sqrt((x - b.x) * (x - b.x) + (y - b.y) * (y - b.y));
  };
  static double DistanceSquared(const Vector2& a, const Vector2& b) {
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
  };
  double DistanceSquared(const Vector2& b) const {
    return (x - b.x) * (x - b.x) + (y - b.y) * (y - b.y);
  };

  static Vector2 normalized(const Vector2& v) { return v.normalized(); };
  Vector2 normalized() const {
    auto magnitude = getMagnitude();

    // If the magnitude is not null
    if (magnitude > 0.)
      return Vector2(x, y) / magnitude;
    else
      return {x, y};
  };

  static const Vector2 zero;
};

const Vector2 Vector2::zero = {0, 0};

struct Boid {
  Boid(const Vector2& pos, const Vector2& vel) : position(pos), velocity(vel){};
  Boid() : position({0, 0}), velocity({0, 0}){};
  Vector2 position;
  Vector2 velocity;
};

struct Cohesion {
  double radius;
  double k;

  Cohesion() = default;

  Vector2 ComputeForce(const vector<Boid>& boids, int boidAgentIndex) {
    Vector2 centerMass = Vector2::zero;
    int numberOfAgents = 0, i;
    // for loop parsing through vector
    for (i = 0; i < boids.size(); i++) {
      // calculating the distance between for loop parsing & main parsing
      double distance = (boids[i].position - boids[boidAgentIndex].position).getMagnitude();
      // if distance is less than or equal to radius and the current index is not including the agent itself
      if (distance <= radius && i != boidAgentIndex) {
        // adding the pos to the centerMass vector & incrementing numberOfAgents
        centerMass += boids[i].position;
        numberOfAgents++;
      }
    }
    // dividing the centerMass vec by the numberOfAgents
    centerMass /= numberOfAgents;
    // calculating the force for each index by removing the vector pos from centerMass
    Vector2 agentForce = centerMass - boids[boidAgentIndex].position;
    // if the magnitude is less than or equal to the radius, if TRUE, the force is divided by radius and multiplied by k, otherwise returns empty vector
    return agentForce.getMagnitude() <= radius ? (agentForce / radius) * k : Vector2::zero;
  }
};

struct Alignment {
  double radius;
  double k;

  Alignment() = default;

  Vector2 ComputeForce(const vector<Boid>& boids, int boidAgentIndex) {
    Vector2 force = Vector2::zero;
    int numberOfAgents = 0, i;
    // same layout as cohesion
    for (i = 0; i < boids.size(); i++) {
      double distance = (boids[i].position - boids[boidAgentIndex].position).getMagnitude();
      // checking if distance is less than or equal to radius without needing to check if radius contains itself
      if (distance <= radius) {
        // collecting the velocity of each boid for later average
        force += boids[i].velocity;
        numberOfAgents++;
      }
    }
    force /= numberOfAgents;
    return force * k;
  }
};

struct Separation {
  double radius;
  double k;
  double maxForce;

  Separation() = default;

  Vector2 ComputeForce(const vector<Boid>& boids, int boidAgentIndex) {
    Vector2 force = Vector2::zero;
    int i;
    // parsing for loop based on boid size
    for (i = 0; i < boids.size(); i++) {
      // if current index is not the agentIndex
      if (i != boidAgentIndex) {
        // calculate the direction of each neighbor and finding the magnitude of the distance
        Vector2 direction = boids[i].position - boids[boidAgentIndex].position;
        double distance = direction.getMagnitude();
        // if distance is less than the radius
        if (distance <= radius) {
          // take force vector and add each instance of the direction divided by the sqr magnitude
          force += direction / direction.sqrMagnitude();
        }
      }
    }
    // ternary operator checking if magnitude is greater than maxForce, if TRUE normalized force * maxForce & k, otherwise force * k
    return force.getMagnitude() > maxForce ? force.normalized() * maxForce * k : force * k;
  }
};

// feel free to edit this main function to meet your needs
int main() {
  // Variable declaration
  Separation separation{};
  Alignment alignment{};
  Cohesion cohesion{};
  int numberOfBoids;
  string line;  // for reading until EOF
  vector<Boid> currentState, newState;
  // Input Reading
  cin >> cohesion.radius >> separation.radius >> separation.maxForce >> alignment.radius
      >> cohesion.k >> separation.k >> alignment.k >> numberOfBoids;
  for (int i = 0; i < numberOfBoids; i++) {
    Boid b;
    cin >> b.position.x >> b.position.y >> b.velocity.x >> b.velocity.y;
    // cout << "b.y: " << b.y << endl;
    currentState.push_back(b);
    newState.push_back(b);
  }
  cin.ignore(256, '\n');
  // Final input reading and processing
  // todo: edit this. probably my code will be different than yours.
  while (getline(cin, line)) {  // game loop
    // Use double buffer! you should read from the current and store changes in the new state.
    currentState = newState;
    double deltaT = stod(line);
    // a vector of the sum of forces for each boid.
    vector<Vector2> allForces = vector<Vector2>(numberOfBoids, {0, 0});
    // Compute Forces
    for (int i = 0; i < numberOfBoids; i++) {   // for every boid
      for (int j = 0; j < numberOfBoids; j++) {  // for every boid combination. Pre-processing loop.
        // Process Cohesion Forces
        auto dist = (currentState[i].position - currentState[j].position).getMagnitude();
        if (i != j && dist <= cohesion.radius) {
          allForces[i] += cohesion.ComputeForce(currentState, i);
        }
        // Process Separation Forces
        if (i != j && dist <= separation.radius) {
          allForces[i] += separation.ComputeForce(currentState, i);
        }
        // Process Alignment Forces
        if (i != j && dist <= alignment.radius) {
          allForces[i] += alignment.ComputeForce(currentState, i);
        }
      }
    }
    // Tick Time and Output
    // todo: edit this. probably my code will be different than yours.
    cout << fixed << setprecision(3);  // set 3 decimal places precision for output
    for (int i = 0; i < numberOfBoids; i++) { // for every boid
      newState[i].velocity += allForces[i] * deltaT;
      newState[i].position += newState[i].velocity * deltaT; // Tristian provided the change from currentState to newState rounds down
      cout << newState[i].position.x << " " << newState[i].position.y << " " << newState[i].velocity.x << " " << newState[i].velocity.y << endl;
    }
  }

  return 0;
}
