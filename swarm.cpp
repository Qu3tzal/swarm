#include <cmath>
#include <string>
#include <vector>
#include <random>
#include <limits>
#include <utility>
#include <sstream>
#include <iostream>

#include <SFML/Graphics.hpp>

/**
 * Constants.
 */
namespace constants {
    const unsigned int TicksPerSecond = 60;
    const sf::Time TimePerTick = sf::seconds(1.f / TicksPerSecond);
    const std::string WindowTitle = "swarm";
    const float ViewSpeed = 300.f;
    const float MinSpeed = 0.1f;
    const float DefaultMaxSpeed = 200.f;
    const float MinMovement = 0.1f;
    const float SeekRelativeDistance = 0.25f;
    const float FleeRelativeDistance = 5.f;
    const float SizeOfAgents = 20.f;
    const float PursueRelativeDistance = 3.f;
    const std::size_t NumberOfWanderingAgents = 20;
    const std::size_t NumberOfPursuingAgents = 10;
    const std::size_t PursueEvadeCircleSize = 10;
}

/**
 * Maths utils.
 */
namespace maths {
    // Float functions.
    inline bool isZero(const float& x) {
        return std::fabs(x) > std::numeric_limits<float>::epsilon();
    }

    inline bool isGreater(const float& x, const float& y) {
        return (x - y) > std::numeric_limits<float>::epsilon();
    }

    inline bool isEqual(const float& x, const float& y) {
        return isZero(x - y);
    }

    // Distance functions.
    inline float squarredEuclidianDistance(const sf::Vector2f& a, const sf::Vector2f& b) {
        return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
    }

    inline float euclidianDistance(const sf::Vector2f& a, const sf::Vector2f& b) {
        return std::sqrt(squarredEuclidianDistance(a, b));
    }

    // Vector manipulation functions.
    inline float length(const sf::Vector2f& x) {
        return std::sqrt(std::pow(x.x, 2) + std::pow(x.y, 2));
    }

    inline sf::Vector2f normalize(const sf::Vector2f& x) {
        return x / length(x);
    }

    inline float dotProduct(const sf::Vector2f& x, const sf::Vector2f& y) {
        return x.x * y.x + x.y * y.y;
    }

	double random() {
		static std::random_device rd;
		static std::mt19937 mt(rd());
		static std::uniform_real_distribution<double> distribution(0.0, 1.0);

		return distribution(mt);
	}
}

/**
 *  Physics.
 */
// The hitbox structures holds the data necessary for a broad and a narrow phase in collision detection.
struct Hitbox {
    // The circle hitbox's radius, used during the broad phase along with the movement's position attribute.
    // The circle hitbox MUST be equal or greater than the convex shape hitbox.
    float radius;

    // The points forming the convex shape of the hitbox, used during the narrow phase.
    std::vector<sf::Vector2f> points;
    // The axes formed by the points. Used by the SAP algorithm during narrow phase of collision detection.
    std::vector<sf::Vector2f> axes;
    sf::Vector2f gravitationalCenter;

    // This method generates the circle hitbox from the convex shape hitbox.
    void computeCircleHitbox() {
        // Compute center of gravity.
        gravitationalCenter = sf::Vector2f(0.f, 0.f);
        for (sf::Vector2f& point : points)
            gravitationalCenter += point;
        gravitationalCenter /= (float)points.size();

        // We increase the radius every time we find a point that is even further from the center.
        // That way our circle hitbox will contain all the points of the convex shape hitbox.
        radius = 0.f;

        for (sf::Vector2f& point : points) {
            float distance = maths::euclidianDistance(gravitationalCenter, point);

            if (distance > radius)
                radius = distance;
        }
    }

    // This method generates the convex shape axes.
    void computeAxes() {
        axes.clear();

        for (std::size_t i(0) ; i < points.size() ; i++) {
			sf::Vector2f a(0.f, 0.f), b(0.f, 0.f);

			if (i == points.size() - 1) {
				a = points[i];
				b = points[0];
			} else {
				a = points[i];
				b = points[i + 1];
			}

            sf::Vector2f c = maths::normalize(b - a);
			sf::Vector2f axe = c;
			axe.x = -c.y;
			axe.y =  c.x;

			axes.push_back(axe);
        }
    }
};

// The movement structure holds the data for the movement integration.
struct Movement {
    float maxSpeed = constants::DefaultMaxSpeed; // The maximum length the velocity vector can reach is called maximum speed.

    sf::Vector2f acceleration;
    sf::Vector2f velocity;
    sf::Vector2f position;
};

// The simple point body structure simulates a single point in regards to physics laws but collides with a convex shape hitbox.
// Applying a force to this body is like applying a force to a single point.
struct SimplePointBody {
    // The hitbox and the movement components.
    Hitbox hitbox;
    Movement movement;

    // Physical properties.
    bool gravity;
    bool moveable;

    SimplePointBody()
        : gravity(true), moveable(true)
    {

    }
};

// The hitbox collision structure holds the data of one detected collision.
struct HitboxCollision {
    std::size_t first; // The index of the first hitbox in the bodies array.
    std::size_t second; // The index of the second hitbox in the bodies array.
    sf::Vector2f projectionAxe; // The axe on which the projection is the shortest (shortest way out of the collision).
};

void integrateMovement(std::vector<SimplePointBody>& bodies, const sf::Time& timeStep) {
    for (SimplePointBody& body : bodies) {
        body.movement.velocity += body.movement.acceleration * timeStep.asSeconds();

        float speed = maths::length(body.movement.velocity);
        if (speed < constants::MinSpeed) {
            body.movement.velocity = sf::Vector2f(0.f, 0.f);
        } else if (speed > body.movement.maxSpeed) {
            body.movement.velocity = maths::normalize(body.movement.velocity) * body.movement.maxSpeed;
        }

        body.movement.position += body.movement.velocity * timeStep.asSeconds();
        body.movement.velocity = sf::Vector2f(0.f, 0.f); // Reset the velocity to prevent the agents from going too far.
        body.movement.acceleration = sf::Vector2f(0.f, 0.f);
    }
}

bool checkCollisionBetweenConvexShapes(const Hitbox& a, const Hitbox& b, sf::Vector2f& projectionVector) {
    std::vector<sf::Vector2f> all_axes;
    all_axes.reserve(a.axes.size() + b.axes.size());
    all_axes.insert(all_axes.end(), a.axes.begin(), a.axes.end());
    all_axes.insert(all_axes.end(), b.axes.begin(), b.axes.end());

    std::unique(all_axes.begin(), all_axes.end());

    projectionVector = sf::Vector2f(0.f, 0.f);

    float projectionVectorLength(std::numeric_limits<float>::max());

    for (sf::Vector2f& axe : all_axes) {
        float a_min_value(std::numeric_limits<float>::max()), a_max_value(std::numeric_limits<float>::min());
        float b_min_value(std::numeric_limits<float>::max()), b_max_value(std::numeric_limits<float>::min());

        for (sf::Vector2f point : a.points) {
            float projection = maths::dotProduct(point, axe) / maths::dotProduct(axe, axe);

            if (projection < a_min_value)
                a_min_value = projection;

            if (projection > a_max_value)
                a_max_value = projection;
        }

        for (sf::Vector2f point : b.points) {
            float projection = maths::dotProduct(point, axe) / maths::dotProduct(axe, axe);

            if (projection < b_min_value)
                b_min_value = projection;

            if (projection > b_max_value)
                b_max_value = projection;
        }

        if (a_min_value <= b_min_value && a_max_value <= b_max_value && a_max_value >= b_min_value) {
            /*
                    _________
                        __________
            */
            sf::Vector2f wayOut = axe * (a_max_value - b_min_value);

            if (maths::length(wayOut) < projectionVectorLength) {
                projectionVector = wayOut;
                projectionVectorLength = maths::length(projectionVector);
            }
        } else if (b_min_value <= a_min_value && b_max_value <= a_max_value && b_max_value >= a_min_value) {
            /*
                        ___________
                    _________
            */
            sf::Vector2f wayOut = axe * (a_min_value - b_max_value);

            if (maths::length(wayOut) < projectionVectorLength) {
                projectionVector = wayOut;
                projectionVectorLength = maths::length(projectionVector);
            }
        } else if (a_min_value <= b_min_value && a_max_value >= b_max_value) {
            /*
                    ________________
                        _____
            */
            float by_right = b_min_value - a_max_value;
            float by_left = b_max_value - a_min_value;

            if (std::abs(by_left) < std::abs(by_right)) {
                if (std::abs(by_left) < projectionVectorLength) {
                    projectionVector = axe * by_left;
                    projectionVectorLength = maths::length(projectionVector);
                }
            } else {
                if (std::abs(by_right) < projectionVectorLength) {
                    projectionVector = axe * by_right;
                    projectionVectorLength = maths::length(projectionVector);
                }
            }
        } else if (b_min_value <= a_min_value && b_max_value >= a_max_value) {
            /*
                        _____
                    ________________
            */
            float by_right = b_min_value - a_max_value;
            float by_left = b_max_value - a_min_value;

            if (std::abs(by_left) < std::abs(by_right)) {
                if (std::abs(by_left) < projectionVectorLength) {
                    projectionVector = axe * by_left;
                    projectionVectorLength = maths::length(projectionVector);
                }
            } else {
                if (std::abs(by_right) < projectionVectorLength) {
                    projectionVector = axe * by_right;
                    projectionVectorLength = maths::length(projectionVector);
                }
            }
        } else {
            /*
                    ___
                        ____
                or
                        ____
                    ___
            */
            projectionVector = sf::Vector2f(0.f, 0.f);
            return false;
        }
    }

    return true;
}

std::vector<HitboxCollision> detectCollisions(std::vector<SimplePointBody>& bodies) {
    // We use indexes on bodies in the bodies array. The body array should not be shrinked nor elements swapped during the collision detection.
    std::vector<std::pair<std::size_t, std::size_t>> broadPhaseCollisions;

    // Broad phase collisions.
    // We use circle-circle collisions.
    for (std::size_t firstBodyIndex(0) ; firstBodyIndex < bodies.size() ; ++firstBodyIndex) {
        for (std::size_t secondBodyIndex(0) ; secondBodyIndex < bodies.size() ; ++secondBodyIndex) {
            // Do not collide a body against itself.
            if (firstBodyIndex == secondBodyIndex)
                continue;

            // Check if the two circle hitboxes collides.
            SimplePointBody fst = bodies[firstBodyIndex];
            SimplePointBody snd = bodies[secondBodyIndex];

            float squarredRadiusSum = std::pow(fst.hitbox.radius + snd.hitbox.radius, 2);
            float squarredDistanceBetweenCenters = maths::squarredEuclidianDistance(fst.hitbox.gravitationalCenter + fst.movement.position, snd.hitbox.gravitationalCenter + snd.movement.position);

            if (squarredDistanceBetweenCenters < squarredRadiusSum) {
                // We have a broad phase collision.
                // We register the pair to check the collision in narrow phase.
                broadPhaseCollisions.push_back({firstBodyIndex, secondBodyIndex});
            }
        }
    }

    // The final collisions array.
    std::vector<HitboxCollision> narrowPhaseCollisions;

    // Narrow phase collisions.
    // We use the SAP theorem to check collisions between convex shapes.
    for (std::pair<std::size_t, std::size_t> collision : broadPhaseCollisions) {
        Hitbox a = bodies[collision.first].hitbox;
        Hitbox b = bodies[collision.second].hitbox;

        for (sf::Vector2f& point : a.points) {
            point += bodies[collision.first].movement.position;
        }

        for (sf::Vector2f& point : b.points) {
            point += bodies[collision.second].movement.position;
        }

        sf::Vector2f projectionAxe;

        if (checkCollisionBetweenConvexShapes(a, b, projectionAxe)) {
            narrowPhaseCollisions.push_back({collision.first, collision.second, projectionAxe});

            if (bodies[collision.first].moveable && bodies[collision.second].moveable) {
                bodies[collision.first].movement.position -= projectionAxe / 2.f;
                bodies[collision.second].movement.position += projectionAxe / 2.f;

                bodies[collision.first].movement.velocity -= projectionAxe / constants::TimePerTick.asSeconds();
                bodies[collision.second].movement.velocity += projectionAxe / constants::TimePerTick.asSeconds();
            } else if (bodies[collision.first].moveable) {
                bodies[collision.first].movement.position -= projectionAxe;
                bodies[collision.first].movement.velocity -= projectionAxe / constants::TimePerTick.asSeconds();
            }
        }
    }

    return narrowPhaseCollisions;
}

/**
 * AI.
 */
enum class SoloAgentBehaviorType {
    Seek,
    Flee,
    Pursue,
    Evade,
    Wander,
    AvoidObstacle
};

enum class SwarmAgentBehaviorType {
    Separate,
    Cohesion,
    Align,
    FollowLeader,
    Queue
};

struct SoloAgentBehavior {
    SoloAgentBehaviorType type;

    std::size_t targetBody; // The ID of the target (only needed for some behaviors).
    sf::Vector2f targetPosition; // The position of the target when its not a body (only needed for some behaviors).
};

struct SwarmAgentBehavior {
    SwarmAgentBehaviorType type;

    std::size_t targetBody; // The ID of the target (only needed for some behaviors).
    sf::Vector2f targetPosition; // The position of the target when its not a body (only needed for some behaviors).
};

// The basic agent class.
class Agent {
    public:
        // Ctor & dtor.
        explicit Agent() {}
        virtual ~Agent() {}

        std::size_t body; // The ID of the agent's body.
};

// The solo agent adopts one or multiple steering behaviors.
class SoloAgent : public Agent {
    public:
        // The behaviors of the agent.
        // Each behavior is associated to a weight.
        std::vector<std::pair<SoloAgentBehavior, float>> behaviors;
};

 // The swarm agent adopts one or multiple flocking behaviors.
class SwarmAgent : public Agent {
    public:
        // The behaviors of the agent.
        // Each behavior is associated to a weight.
        std::vector<std::pair<SwarmAgentBehavior, float>> behaviors;

        int swarmId; // The ID of the swarm this swarm agent belongs to.
};

// This function calls the behavior function for each solo agent.
void simulateSoloBehaviors(std::vector<SoloAgent>& agents, std::vector<SimplePointBody>& bodies) {
    for (SoloAgent& agent : agents) {
        for (std::pair<SoloAgentBehavior, float>& behaviorPair : agent.behaviors) {
            SoloAgentBehavior& behavior = behaviorPair.first;

            if (behavior.type == SoloAgentBehaviorType::Seek) {
                /**
                 * For the seek behavior, we want the agent to follow the target.
                 */
                sf::Vector2f bodyCenter = bodies[agent.body].movement.position + bodies[agent.body].hitbox.gravitationalCenter;
                float bodyRadius = bodies[agent.body].hitbox.radius;

                // Compute the distance to the target. We only seek if the distance is big (big being a constant factor times the body radius).
                // This is for demonstration purposes, because we don't want the agents to wiggle when they reach the seeking position.
                if (maths::length(behavior.targetPosition - bodyCenter) > constants::SeekRelativeDistance * bodyRadius) {
                    sf::Vector2f movement = bodies[agent.body].movement.maxSpeed * maths::normalize(behavior.targetPosition - bodyCenter);

                    // Only apply the movement if it is big enough to prevent wiggles.
                    if (maths::isGreater(maths::length(movement), constants::MinMovement))
                        bodies[agent.body].movement.velocity += movement;
                }
            } else if (behavior.type == SoloAgentBehaviorType::Flee) {
                /**
                 * For the flee behavior, we want the agent to follow the target.
                 */
                sf::Vector2f bodyCenter = bodies[agent.body].movement.position + bodies[agent.body].hitbox.gravitationalCenter;
                float bodyRadius = bodies[agent.body].hitbox.radius;

                // Compute the distance to the target. We only flee if the distance is small (small being a constant factor times the body radius).
                if (maths::length(behavior.targetPosition - bodyCenter) < constants::FleeRelativeDistance * bodyRadius) {
                    sf::Vector2f movement = bodies[agent.body].movement.maxSpeed * maths::normalize(behavior.targetPosition - bodyCenter);

                    bodies[agent.body].movement.velocity -= movement;
                }
            } else if (behavior.type == SoloAgentBehaviorType::Pursue) {
                /**
                 * For the pursue behavior, we want the agent to follow the target body.
                 */
                // Get the position of the body we want to pursue.
                sf::Vector2f targetPosition = bodies[behavior.targetBody].movement.position + bodies[behavior.targetBody].hitbox.gravitationalCenter;

                sf::Vector2f bodyCenter = bodies[agent.body].movement.position + bodies[agent.body].hitbox.gravitationalCenter;
                float bodyRadius = bodies[agent.body].hitbox.radius;

                // Compute the distance to the target. We only pursue if the distance is big (big being a constant factor times the body radius).
                // This is for demonstration purposes, because we don't want the agents to block each other.
                if (maths::length(targetPosition - bodyCenter) > constants::PursueRelativeDistance * bodyRadius) {
                    sf::Vector2f movement = bodies[agent.body].movement.maxSpeed * maths::normalize(targetPosition - bodyCenter);

                    // Only apply the movement if it is big enough to prevent wiggles.
                    if (maths::isGreater(maths::length(movement), constants::MinMovement))
                        bodies[agent.body].movement.velocity += movement;
                }
            } else if (behavior.type == SoloAgentBehaviorType::Evade) {
                /**
                 * For the evade behavior, we want the agent to follow the target body.
                 */
                // Get the position of the body we want to evade.
                sf::Vector2f targetPosition = bodies[behavior.targetBody].movement.position + bodies[behavior.targetBody].hitbox.gravitationalCenter;

                sf::Vector2f bodyCenter = bodies[agent.body].movement.position + bodies[agent.body].hitbox.gravitationalCenter;
                float bodyRadius = bodies[agent.body].hitbox.radius;

                // Compute the distance to the target. We only evade if the distance is small (small being a constant factor times the body radius).
                if (maths::length(targetPosition - bodyCenter) < constants::FleeRelativeDistance * bodyRadius) {
                    sf::Vector2f movement = bodies[agent.body].movement.maxSpeed * maths::normalize(targetPosition - bodyCenter);

                    bodies[agent.body].movement.velocity -= movement;
                }
            } else if (behavior.type == SoloAgentBehaviorType::Wander) {
            	/**
            	 * For the wander behavior, we add a small offset to the velocity.
            	 * The previous velocity is reminded in the target position attribute.
            	 */
	            sf::Vector2f offset = bodies[agent.body].movement.maxSpeed * sf::Vector2f(maths::random() - 0.5f, maths::random() - 0.5f);

	            bodies[agent.body].movement.velocity += behavior.targetPosition + offset;
	            bodies[agent.body].movement.velocity = bodies[agent.body].movement.maxSpeed * maths::normalize(bodies[agent.body].movement.velocity);

	            behavior.targetPosition = bodies[agent.body].movement.velocity;
            } else if (behavior.type == SoloAgentBehaviorType::AvoidObstacle) {

            }
        }
    }
}

// This function calls the behavior function for each swarm agent.
void simulateSwarmBehaviors(std::vector<SwarmAgent>& agents, std::vector<SimplePointBody>& bodies) {
    for (SwarmAgent& agent : agents) {
        for (const std::pair<SwarmAgentBehavior, float>& behaviorPair : agent.behaviors) {
            const SwarmAgentBehavior& behavior = behaviorPair.first;

            if (behavior.type == SwarmAgentBehaviorType::Separate) {

            } else if (behavior.type == SwarmAgentBehaviorType::Cohesion) {

            } else if (behavior.type == SwarmAgentBehaviorType::Align) {

            } else if (behavior.type == SwarmAgentBehaviorType::FollowLeader) {

            } else if (behavior.type == SwarmAgentBehaviorType::Queue) {

            }
        }
    }
}

/**
 *  Rendering.
 */
// This function renders a list of bodies.
void renderBodies(sf::RenderTarget& target, std::vector<SimplePointBody>& bodies) {
    std::size_t verticesCount(0);

    // Count the number of vertices necessary.
    for (SimplePointBody& body : bodies) {
        verticesCount += body.hitbox.points.size() * 2;
    }

    // Create the vertex array.
    sf::VertexArray vertexArray(sf::PrimitiveType::Lines, verticesCount);

    // Fill the vertex array.
    std::size_t bodyOffset = 0;
    for (SimplePointBody& body : bodies) {
        for (std::size_t i(0) ; i < body.hitbox.points.size() ; ++i) {
            // Remember to take into account the world transformation of each point.
            vertexArray[bodyOffset + i * 2].position = body.movement.position + body.hitbox.points[i];
            vertexArray[bodyOffset + i * 2].color = sf::Color::Green;

            // Connect the last point to the first one.
            if (i == body.hitbox.points.size() - 1) {
                vertexArray[bodyOffset + i * 2 + 1].position = body.movement.position + body.hitbox.points[0];
            } else {
                vertexArray[bodyOffset + i * 2 + 1].position = body.movement.position + body.hitbox.points[i + 1];
            }

            vertexArray[bodyOffset + i * 2 + 1].color = sf::Color::Green;
        }

        // Increase the offset in the vertex array by the number of vertices used by the current body.
        bodyOffset += body.hitbox.points.size() * 2;
    }

    // Draw the vertex array.
    target.draw(vertexArray);
}

/**
 * Performances.
 */
// Simple metrics class to group up the metrics variables.
struct PerformanceMetrics {
    int frameCount;
    int tickCount;

    PerformanceMetrics() : frameCount(0), tickCount(0) {

    }

    void reset() {
        frameCount = 0;
        tickCount = 0;
    }
};

// Program start.
int main(int argc, char *argv[]) {
    // Init graphics.
    sf::RenderWindow window(sf::VideoMode(1280, 720), constants::WindowTitle);
    sf::View view = window.getView();
    view.setCenter(500, 500);
    window.setView(view);

    // Init random number generator.


    // Init world's physics.
    // Contains the agents bodies arranged in the given order :
    //      the first agent
    //      the fleeing agents
    //      the pursuing agents
    //      the pursuing-evading agents
    std::vector<SimplePointBody> bodies;

    {
        SimplePointBody agentBody;
        agentBody.hitbox.points.push_back(sf::Vector2f(0, 0));
        agentBody.hitbox.points.push_back(sf::Vector2f(0, constants::SizeOfAgents));
        agentBody.hitbox.points.push_back(sf::Vector2f(constants::SizeOfAgents, constants::SizeOfAgents));
        agentBody.hitbox.points.push_back(sf::Vector2f(constants::SizeOfAgents, 0));

        // Compute the hitbox's axes and circle.
        agentBody.hitbox.computeAxes();
        agentBody.hitbox.computeCircleHitbox();

        bodies.push_back(agentBody);
    }

    for (std::size_t i(0) ; i < constants::NumberOfWanderingAgents ; ++i) {
        SimplePointBody agentBody;
        agentBody.hitbox.points.push_back(sf::Vector2f(0, 0));
        agentBody.hitbox.points.push_back(sf::Vector2f(0, constants::SizeOfAgents));
        agentBody.hitbox.points.push_back(sf::Vector2f(constants::SizeOfAgents, constants::SizeOfAgents));
        agentBody.hitbox.points.push_back(sf::Vector2f(constants::SizeOfAgents, 0));

        // Compute the hitbox's axes and circle.
        agentBody.hitbox.computeAxes();
        agentBody.hitbox.computeCircleHitbox();

        // Random position.
        agentBody.movement.position.x = 500.f + (maths::random() - 0.5f) * 500.f;
        agentBody.movement.position.y = 500.f + (maths::random() - 0.5f) * 500.f;

        bodies.push_back(agentBody);
    }

    // Create the pursuing agents as a line.
    for (std::size_t i(0) ; i < constants::NumberOfPursuingAgents ; ++i) {
        SimplePointBody agentBody;
        agentBody.hitbox.points.push_back(sf::Vector2f(0, 0));
        agentBody.hitbox.points.push_back(sf::Vector2f(0, constants::SizeOfAgents));
        agentBody.hitbox.points.push_back(sf::Vector2f(constants::SizeOfAgents, constants::SizeOfAgents));
        agentBody.hitbox.points.push_back(sf::Vector2f(constants::SizeOfAgents, 0));

        // Compute the hitbox's axes and circle.
        agentBody.hitbox.computeAxes();
        agentBody.hitbox.computeCircleHitbox();

        // Random position.
        agentBody.movement.position.x = 100.f + 100.f * i;
        agentBody.movement.position.y = 200.f;

        bodies.push_back(agentBody);
    }

    // Create the evading agents as a circle.
    for (std::size_t i(0) ; i < constants::PursueEvadeCircleSize ; ++i) {
        const sf::Vector2f center(500.f, 500.f);
        const float radius = 150.f;

        SimplePointBody agentBody;
        agentBody.hitbox.points.push_back(sf::Vector2f(0, 0));
        agentBody.hitbox.points.push_back(sf::Vector2f(0, constants::SizeOfAgents));
        agentBody.hitbox.points.push_back(sf::Vector2f(constants::SizeOfAgents, constants::SizeOfAgents));
        agentBody.hitbox.points.push_back(sf::Vector2f(constants::SizeOfAgents, 0));

        // Compute the hitbox's axes and circle.
        agentBody.hitbox.computeAxes();
        agentBody.hitbox.computeCircleHitbox();

        // Random position.
        agentBody.movement.position.x = center.x + radius * std::cos(i * 2.f * M_PI / constants::PursueEvadeCircleSize);
        agentBody.movement.position.y = center.y + radius * std::sin(i * 2.f * M_PI / constants::PursueEvadeCircleSize);

        bodies.push_back(agentBody);
    }

    // Init AIs.
    std::vector<SoloAgent> soloAgents;
    std::vector<SwarmAgent> swarmAgents;

    // Create a solo agent that seeks the cursor.
    {
        SoloAgentBehavior behavior;
        behavior.type = SoloAgentBehaviorType::Seek;

        SoloAgent agent;
        agent.body = 0;
        agent.behaviors.push_back(std::make_pair(behavior, 1.f));

        // Add the agent to the list of the solo agents.
        soloAgents.push_back(agent);
    }

    // Create solo agents that flee the cursor.
    for (std::size_t i(0) ; i < constants::NumberOfWanderingAgents ; ++i) {
        SoloAgentBehavior behavior;
        behavior.type = SoloAgentBehaviorType::Wander;

	    // Random velocity.
	    sf::Vector2f velocity = sf::Vector2f(maths::random(), maths::random());
	    behavior.targetPosition = velocity;

        SoloAgent agent;
        agent.body = i + 1;
        agent.behaviors.push_back(std::make_pair(behavior, 1.f));

        // Add the agent to the list of the solo agents.
        soloAgents.push_back(agent);
    }

    // Create a chain of solo agents that pursue one another.
    // The first of these pursuing agents pursues the agent that seek the cursor.
    for (std::size_t i(0) ; i < constants::NumberOfPursuingAgents ; ++i) {
        SoloAgentBehavior behavior;
        behavior.type = SoloAgentBehaviorType::Pursue;

        // The first agent of the chain pursues the body of the very first solo agent.
        if (i == 0)
            behavior.targetBody = 0;
        else
            behavior.targetBody = 1 + constants::NumberOfWanderingAgents + i - 1; // This is a little dirty. The offset of the bodies of the pursuing agents is : the first agent + the fleeing agents.

        SoloAgent agent;
        agent.body = i + 1 + constants::NumberOfWanderingAgents;
        agent.behaviors.push_back(std::make_pair(behavior, 1.f));

        // Add the agent to the list of the solo agents.
        soloAgents.push_back(agent);
    }

    // Create a circle of solo agents that evade one another.
    for (std::size_t i(0) ; i < constants::PursueEvadeCircleSize ; ++i) {
        const std::size_t offset = 1 + constants::NumberOfWanderingAgents + constants::NumberOfPursuingAgents;

        SoloAgentBehavior pursueBehavior, evadeBehavior;
        pursueBehavior.type = SoloAgentBehaviorType::Pursue;
        evadeBehavior.type = SoloAgentBehaviorType::Evade;

        if (i == 0) {
            // Pursue the last, evade the next.
            pursueBehavior.targetBody = offset + constants::PursueEvadeCircleSize - 1;
            evadeBehavior.targetBody = offset + i + 1;
        } else if (i == constants::PursueEvadeCircleSize - 1) {
            // Pursue the previous, evade the first.
            pursueBehavior.targetBody = offset + i - 1;
            evadeBehavior.targetBody = offset;
        } else {
            // Pursue the previous, evade the next.
            // This is a little dirty.
            // The offset of the bodies of the pursuing agents is : the first agent + the fleeing agents + the pursuing agents.
            pursueBehavior.targetBody = offset + i - 1;
            evadeBehavior.targetBody = offset + i + 1;
        }

        SoloAgent agent;
        agent.body = i + 1 + constants::NumberOfWanderingAgents + constants::NumberOfPursuingAgents;
        agent.behaviors.push_back(std::make_pair(pursueBehavior, 0.5f));
        agent.behaviors.push_back(std::make_pair(evadeBehavior, 0.5f));

        // Add the agent to the list of the solo agents.
        soloAgents.push_back(agent);
    }

    // Init time management.
    sf::Clock gameClock;
    sf::Time timeElapsedSinceLastGameUpdate(sf::Time::Zero), timeElapsedSinceLastMetricsUpdate(sf::Time::Zero);

    // Init metrics.
    PerformanceMetrics metrics;

    // Main loop.
    while (window.isOpen()) {
        // Time management.
        sf::Time dt = gameClock.restart();
        timeElapsedSinceLastGameUpdate += dt;
        timeElapsedSinceLastMetricsUpdate += dt;

        // Event handling.
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        // Logic update.
        if (timeElapsedSinceLastGameUpdate >= constants::TimePerTick) {
            timeElapsedSinceLastGameUpdate -= constants::TimePerTick;

            // Check for input to move the view.
            sf::Vector2f movement(0.f, 0.f);
            bool isMovingView(false);

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q)) {
                isMovingView = true;
                movement.x -= 1.f;
            } else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
                isMovingView = true;
                movement.x += 1.f;
            }

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Z)) {
                isMovingView = true;
                movement.y -= 1.f;
            } else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
                isMovingView = true;
                movement.y += 1.f;
            }

            // We use a flag to check if the view has moved to prevent the view from jiggling due to float computing imprecision (0.f may not be exactly 0).
            if (isMovingView) {
                movement *= constants::ViewSpeed * constants::TimePerTick.asSeconds();

                sf::View view = window.getView();
                view.setCenter(view.getCenter() + movement);
                window.setView(view);
            }

            // Update the target position for the agents.
            sf::Vector2f cursorPosition = window.mapPixelToCoords(sf::Mouse::getPosition(window));

            // Only the first agent needs to have his target position updated as the mouse cursor position.
			soloAgents[0].behaviors[0].first.targetPosition = cursorPosition;


            // Simulate the behaviors.
            simulateSoloBehaviors(soloAgents, bodies);
            simulateSwarmBehaviors(swarmAgents, bodies);

            // Integrate the movement.
            integrateMovement(bodies, constants::TimePerTick);

            // Detect collisions.
            std::vector<HitboxCollision> collisions = detectCollisions(bodies);

            metrics.tickCount++;
        }

        // Update the window title if necessary.
        if (timeElapsedSinceLastMetricsUpdate >= sf::seconds(1.f)) {
            // Build title string.
            std::stringstream ss("");
            ss << constants::WindowTitle << " - tps: " << metrics.tickCount << ", fps: " << metrics.frameCount;

            // Set the window title and reset the metrics and the metrics' time counter.
            window.setTitle(ss.str());
            metrics.reset();
            timeElapsedSinceLastMetricsUpdate -= sf::seconds(1.f);
        }

        // Rendering.
        window.clear();
        renderBodies(window, bodies);
        window.display();
        metrics.frameCount++;
    }
}
