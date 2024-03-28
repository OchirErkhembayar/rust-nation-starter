mod cheats;

use hs_hackathon::prelude::*;
use std::time::Duration;

use cheats::angles::Vector;
use cheats::approaching::Hint;
use cheats::positioning::Position;
use cheats::TeamColors;

const CAR: Color = Color::Red;
const TARGET: Color = Color::Blue;

#[allow(unused)]
struct MapState {
    car: Position,
    target: Position,
}

#[allow(unused)]
impl MapState {
    pub async fn infer(drone: &mut Camera) -> eyre::Result<Self> {
        unimplemented!()
    }

    async fn car_orientation(
        current: Position,
        drone: &mut Camera,
        motor: &mut MotorSocket,
        wheels: &mut WheelOrientation,
    ) -> eyre::Result<Vector> {
        unimplemented!()
    }
}

#[derive(Debug)]
#[allow(unused)]
enum State {
    /// Turn the cars direction by doing consecutive front and back movements
    /// until the angle between the cars orientation and the target converges to be under
    /// a specified threshold
    Turning,
    /// Approach the car by doing incremental actions of approaching and measuring interleaved.
    /// So we approach the target a bit, measure if we decreased the distance, if yes repeat, if no
    /// then calibrate. We do this until we hit the target.
    Approaching,
    /// Simply idling on the target and identifying when the target moves away from our current
    /// position.
    Idle,
}

impl State {
    async fn execute(
        &mut self,
        drone: &mut Camera,
        motor: &mut MotorSocket,
        wheels: &mut WheelOrientation,
    ) -> eyre::Result<()> {
        let team_colors = TeamColors {
            car: CAR,
            target: TARGET,
        };
        match self {
            State::Turning => loop {
                let (initial_car, _initial_target) =
                    cheats::internal::infer(&team_colors, drone).await?;

                wheels.set(Angle::straight()).await?;
                motor
                    .move_for(Velocity::forward(), Duration::from_millis(1500))
                    .await?;

                let (new_car, target) = cheats::internal::infer(&team_colors, drone).await?;

                let initial_car_center = Position::from(initial_car);
                let new_car_center = Position::from(new_car);

                let target_center = Position::from(target);

                let car_heading = Vector::from((initial_car_center, new_car_center));
                let desired_angle = Vector::from((new_car_center, target_center));

                let difference_angle = car_heading.angle(desired_angle);

                // The angle will be between -180 and 180
                const STRAIGHT_THRESHOLD: f64 = 10.0;
                const MAX_TURN_ANGLE: f64 = 30.0;

                if difference_angle.abs() <= STRAIGHT_THRESHOLD {
                    // We're straight enough. Just call it a success. We'll need to tune the magic
                    // threshold though.
                    *self = Self::Approaching;
                    continue;
                }

                let new_wheel_angle = if difference_angle.abs() > MAX_TURN_ANGLE {
                    if difference_angle < 0.0 {
                        Angle::left()
                    } else {
                        Angle::right()
                    }
                } else {
                    let raw_angle_value = if difference_angle < 0.0 {
                        linear_map(difference_angle, -MAX_TURN_ANGLE, 0.0, -1.0, 0.0)
                    } else {
                        linear_map(difference_angle, 0.0, MAX_TURN_ANGLE, 0.0, 1.0)
                    };
                    Angle::try_from(raw_angle_value as f32)?
                };

                wheels.set(new_wheel_angle).await?;

                *self = Self::Approaching;
            },
            State::Approaching => {
                let hint = cheats::approaching::auto(
                    &TeamColors {
                        car: CAR,
                        target: TARGET,
                    },
                    drone,
                    motor,
                    wheels,
                )
                .await?;

                *self = match hint {
                    Hint::TargetWasHit => Self::Idle,
                    Hint::OrientationIsOff => Self::Turning,
                };
            }
            State::Idle => {
                cheats::idling::auto(
                    &TeamColors {
                        car: CAR,
                        target: TARGET,
                    },
                    drone,
                    motor,
                    wheels,
                )
                .await?;

                *self = Self::Turning;
            }
        }

        Ok(())
    }
}

fn linear_map(number: f64, in_min: f64, in_max: f64, out_min: f64, out_max: f64) -> f64 {
    (number - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

#[hs_hackathon::main]
async fn main() -> eyre::Result<()> {
    let mut wheels = WheelOrientation::new().await?;
    let mut motor = MotorSocket::open().await?;
    let mut drone = Camera::connect().await?;

    let mut machine = State::Turning;

    loop {
        if let Err(error) = machine.execute(&mut drone, &mut motor, &mut wheels).await {
            tracing::error!(error = display(error), "Something's wrong!");
        }
        tracing::debug!("{:?}", machine);
    }
}
