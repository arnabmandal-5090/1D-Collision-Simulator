#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <algorithm>
#include <iomanip>

using namespace std;

struct Ball {
    string id;
    double mass;
    double pos;
    double vel;
    double last_update_time;
    int version;

    Ball(string id, double m, double p, double v) 
        : id(id), mass(m), pos(p), vel(v), last_update_time(0.0), version(0) {}

    double get_position_at(double t) const {
        return pos + vel * (t - last_update_time);
    }

    void update_state(double t) {
        pos = get_position_at(t);
        last_update_time = t;
        version++;
    }
};

struct Event {
    double time;
    int left_idx;
    int right_idx;
    int left_ver;
    int right_ver;

    Event(double t, int l_idx, int r_idx, int l_ver, int r_ver)
        : time(t), left_idx(l_idx), right_idx(r_idx), left_ver(l_ver), right_ver(r_ver) {}

    bool operator>(const Event& other) const {
        return time > other.time;
    }
};

class Simulator {
private:
    vector<Ball> balls;
    priority_queue<Event, vector<Event>, greater<Event>> pq;

    void predict_collision(int i, int j) {
        if (i < 0 || j >= balls.size()) return;

        Ball& ball_a = balls[i];
        Ball& ball_b = balls[j];

        if (ball_a.vel <= ball_b.vel) return;

        double sync_time = max(ball_a.last_update_time, ball_b.last_update_time);
        double pos_a = ball_a.get_position_at(sync_time);
        double pos_b = ball_b.get_position_at(sync_time);

        double dist = pos_b - pos_a;
        double rel_vel = ball_a.vel - ball_b.vel;

        double dt = (dist > 0) ? (dist / rel_vel) : 0.0;
        double collision_time = sync_time + dt;

        pq.push(Event(collision_time, i, j, ball_a.version, ball_b.version));
    }

public:
    Simulator(vector<Ball>& input_balls) {
        balls = input_balls;
        
        sort(balls.begin(), balls.end(), [](const Ball& a, const Ball& b) {
            return a.pos < b.pos;
        });
    }

    void simulate(int K) {
        for (int i = 0; i < (int)balls.size() - 1; ++i) {
            predict_collision(i, i + 1);
        }

        int collisions_processed = 0;

        while (!pq.empty() && collisions_processed < K) {
            Event event = pq.top();
            pq.pop();

            Ball& ball_a = balls[event.left_idx];
            Ball& ball_b = balls[event.right_idx];

            if (ball_a.version != event.left_ver || ball_b.version != event.right_ver) {
                continue; 
            }

            double t = event.time;
            
            ball_a.update_state(t);
            ball_b.update_state(t);

            double m1 = ball_a.mass, m2 = ball_b.mass;
            double v1 = ball_a.vel, v2 = ball_b.vel;

            ball_a.vel = ((m1 - m2) * v1 + 2 * m2 * v2) / (m1 + m2);
            ball_b.vel = ((m2 - m1) * v2 + 2 * m1 * v1) / (m1 + m2);

            collisions_processed++;
            cout << "Collision " << collisions_processed 
                 << " at t=" << fixed << setprecision(4) << t 
                 << "s: Ball " << ball_a.id << " and Ball " << ball_b.id << "\n";

            predict_collision(event.left_idx - 1, event.left_idx);
            predict_collision(event.right_idx, event.right_idx + 1);
        }
    }
};

int main() {
    vector<Ball> initial_balls = {
        Ball("A", 1.0, 0.0, 2.0),
        Ball("B", 1.0, 5.0, -1.0),
        Ball("C", 2.0, 10.0, -3.0)
    };

    Simulator sim(initial_balls);
    
    cout << "Starting simulation...\n";
    sim.simulate(5);

    return 0;
}
