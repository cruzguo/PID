######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################
import sys

import argparse

import numpy as np
import unittest

from scipy.integrate import odeint
from typing import Callable, Tuple, Dict, Optional

import TestCases
from FlightPlan import FlightPlan

try:
    from RocketPIDStudent_submission import pressure_pd_solution as student_pressure_pd_solution
    from RocketPIDStudent_submission import rocket_pid_solution as student_rocket_pid_solution
    from RocketPIDStudent_submission import who_am_i

    studentExc = None

except Exception as e:
    studentExc = e

# set to True for to display graphical output
SHOW_GRAPH = False
# SHOW_GRAPH = True
try:
    from FlightData.MatPlotLibVisualizer import MatPlotLibVisualizer

    flight_graph = MatPlotLibVisualizer
except ImportError:
    print('MatPlotLib module not found, using turtle graphics.')

    from FlightData.TurtleVisualizer import TurtleVisualizer

    flight_graph = TurtleVisualizer

PRESSURE_PERCENT = 0.35 # part A percent
PRESSURE_SCORE = 100

ROCKET_PERCENT = 0.65 # part B percent
ROCKET_FLIGHT_SCORE = 65.0
ROCKET_LAND_SCORE = 35.0



class PressurePD(object):

    def __init__(self):
        # Standard values and variables
        self.max_flow_delta = 10.0
        self.consumption_rate = 5.0
        self.initial_level = 10.0
        self.max_level = 105.0
        self.min_level = 0.0

        # Workspace
        self.time_final = 300
        self.time_steps = 301

    def start_launch(self, target_pressure: float, pressure_pd_solution: Callable,
                    optional_data: Optional[Dict] = None) -> Tuple[float, str]:
        """
        Start launch to begin consuming fuel for lift off.

        Args:
            target_pressure: The target pressure to maintain
            pressure_pd_solution: Fuel tank pressure PD function to control supply system valve
            optional_data: Optional params to be passed to the pd solution

        Returns:
              Final grade, Output of launch
        """
        output = ''
        current_pressure = self.initial_level
        adjust_log = np.zeros(self.time_steps)
        pressure_log = np.zeros(self.time_steps)
        pressure_change = 0.0

        # deltaT is 1 if time + 1 = time_step
        delta_t = self.time_final // (self.time_steps - 1)
        time = np.linspace(0, self.time_final, self.time_steps)

        # Initialize data
        data = {'ErrorP': 0,
                'ErrorI': 0,
                'ErrorD': 0}

        if optional_data:
            data.update(optional_data)

        for time_steps in range(len(time)):
            pressure_adjust, data = pressure_pd_solution(delta_t, current_pressure, target_pressure, data)

            pressure_adjust = min(pressure_adjust, 1.0)
            pressure_adjust = max(pressure_adjust, -1.0)

            adjust_log[time_steps] = pressure_adjust

            pressure_change += pressure_adjust
            pressure_change = min(pressure_change, self.max_flow_delta)
            pressure_change = max(pressure_change, -self.max_flow_delta)

            current_pressure += pressure_change
            current_pressure -= self.consumption_rate

            pressure_log[time_steps] = current_pressure

        # Plotting for testing purposes
        if SHOW_GRAPH:
            try:
                graph = flight_graph("Fuel tank pressure", 800, 200, 2)

                sub_data_0 = [{'x': time,
                               'y': pressure_log,
                               'color': "#0000ff",
                               'label': "Fuel tank pressure"},
                              {'x': time,
                               'y': [target_pressure] * len(time),
                               'color': "#888800",
                               'label': "Optimal level"}]
                graph.add_plot("Output (%)", sub_data_0)

                sub_data_1 = [{'x': time,
                               'y': adjust_log,
                               'color': "#ff0000",
                               'label': "Input adjustments"}]
                graph.add_plot("Output (%)", sub_data_1)

                graph.done()

            except Exception as exp:
                import traceback
                output += 'Error with plotting results:' + str(exp)
                output += traceback.format_exc()
                output += '\n'

        # Generate scoring
        min_pressure_level = np.min(pressure_log)
        max_pressure_level = np.max(pressure_log)

        if min_pressure_level < self.min_level:
            output += 'Fuel tank pressure level dropped below safe minimum values.\n'
            score = 0.0

        elif max_pressure_level > self.max_level:
            output += 'Fuel tank pressure exceeded maximum design limits.\n'
            score = 0.0

        else:
            start_position = 55
            tolerance = 1.0
            lower_bounds = target_pressure - tolerance
            upper_bounds = target_pressure + tolerance
            pressure_data = pressure_log[start_position:self.time_steps]
            correct = len(np.where(np.logical_and(pressure_data <= upper_bounds, pressure_data >= lower_bounds))[0])
            total_positions = self.time_steps - start_position
            score = (correct / float(total_positions)) * PRESSURE_SCORE

        return score, output


class RocketPID(object):
    """
    Attributes:
        force_propulsion: kn maximum thrust of engines in N
        rho: density of air in kg/km3
        cd: air drag coefficient in unit-less
        area: area of rocket cross section in km2
        vehicle: nominal weight of rocket in kg
        standard_gravity: gravity w/r to altitude in km/s2
        time_final: final number of time steps
        total_steps: total number of time steps
        thrust: thrust log for each time step
        gravity: gravity log for each time step
        drag: drag log for each time step
    """

    def __init__(self):
        # Standard values and variables
        self.force_propulsion = 4000
        self.rho = 1225000000  #
        self.cd = 0.5  #
        self.area = 0.000016  #
        self.vehicle = 50000  #
        self.standard_gravity = 0.00981  #

        # Workspace
        self.time_final = 600
        self.total_steps = 601

        self.thrust = np.zeros(self.total_steps)
        self.gravity = np.zeros(self.total_steps)
        self.drag = np.zeros(self.total_steps)

    def rocket(self, velocity: float, full_time: np.ndarray, instance_count: int, throttle: float,
               fuel: float) -> float:
        """
        Models rocket velocity.

        Args:
            velocity: Current rocket velocity.
            full_time: Full list of time steps.
            instance_count: Current time step in launch.
            throttle: Current throttle value.
            fuel: Current fuel value.

        Returns:
            Change in velocity as float.
        """

        # Force Equations
        mass = self.vehicle + max(0.0, fuel)
        thrust_force = self.force_propulsion * throttle
        gravity_force = self.standard_gravity * mass
        drag_force = 0.5 * self.rho * self.cd * self.area * velocity ** 2

        if velocity < 0:
            drag_force = -drag_force
        if fuel < 0:
            thrust_force = 0

        # Store for plotting
        self.thrust[instance_count + 1] = thrust_force
        self.gravity[instance_count + 1] = abs(gravity_force)
        self.drag[instance_count + 1] = abs(drag_force)

        # First Order Equation for Solving Change in Velocity
        d_vdt = (thrust_force - gravity_force - drag_force) / mass

        return d_vdt

    def launch_rocket(self, flight_plan: FlightPlan, fuel: int, rocket_pid_solution: Callable,
                      optional_data: Optional[Dict] = None) -> Tuple[float, float, float, str]:
        """
        Launch rocket to attempt to fly optimal flight path

        Args:
            flight_plan: Expected flight plan of rocket
            fuel: Fuel load in kg
            rocket_pid_solution: Rocket PID function to control launch
            optional_data: Optional params to be passed to the pd solution

        Returns:
              Final grade, Output of launch
        """
        output = ''

        init_fuel = fuel  # fuel load in kg
        # initial velocity level (height = 0 at base) in km/2
        init_velocity = 0
        # initial engine position (shutoff = 0, max thrust = 1) in percent
        # kerosene RG-1 consumption in kg/s
        fuel_consumption = 480
        # status indicator for landing
        landed = 0
        # status indicator for fuel tank
        fuel_empty = 0
        # status indicator for successful landing
        good_landing = 0

        delta_t = self.time_final // (self.total_steps - 1)
        time = np.linspace(0, self.time_final, self.total_steps)

        throttle_set = np.zeros(self.total_steps)
        velocity_log = np.zeros(self.total_steps)
        optimal_velocity_log = np.zeros(self.total_steps)
        height = np.zeros(self.total_steps)
        fuel_level = np.zeros(self.total_steps)

        # Initialize data
        data = {'ErrorP': 0,
                'ErrorI': 0,
                'ErrorD': 0}

        if optional_data:
            data.update(optional_data)

        # Rocket Altitude ODE solver
        for time_step in range(len(time) - 1):
            if landed > 0:
                break

            (optimal_velocity, is_velocity_mandatory,
             desired_height, is_height_mandatory) = flight_plan.get_current_values(time_step)

            init_throttle, data = rocket_pid_solution(delta_t, velocity_log[time_step], optimal_velocity, data)
            init_throttle = max(0.0, min(1.0, init_throttle))

            # simulate air density drop with altitude
            self.rho = 1225000000 * np.exp(-height[time_step] / 1000)

            # shutoff engines if fuel empty
            if fuel_empty == 1:
                output += 'The rocket ran out of fuel!\n'
                init_throttle = 0

            # ODE solver to simulate rocket velocity change
            rocket_velocity = odeint(self.rocket, init_velocity, [time[time_step], time[time_step + 1]],
                                     args=(time_step, init_throttle, init_fuel))

            # update velocity with ODE value
            init_velocity = rocket_velocity[1][0]
            velocity_log[time_step + 1] = init_velocity  # log current velocity
            throttle_set[time_step + 1] = init_throttle  # log throttle
            init_fuel = init_fuel - fuel_consumption * init_throttle  # reduce fuel per consumption rate
            # log optimal velocity
            optimal_velocity_log[time_step + 1] = optimal_velocity

            # Altitude and Fuel Checks
            if height[time_step] < 0 and abs(init_velocity) > 0.11:
                height[time_step + 1] = 0
                landed = time_step
                output += 'The rocket CRASHED!\n'
            elif height[time_step] < 0 and abs(init_velocity) <= 0.11 and time_step > 10:
                height[time_step + 1] = 0
                landed = time_step
                good_landing = True
                output += 'The rocket landed safely!\n'
            elif height[time_step] >= 0:
                height[time_step + 1] = height[time_step] + init_velocity * delta_t

            if fuel_empty == 1:
                fuel_level[time_step + 1] = 0
            elif init_fuel < 0:
                fuel_level[time_step + 1] = 0
                fuel_empty = 1
            else:
                fuel_level[time_step + 1] = init_fuel

        # Plotting for testing purposes
        if SHOW_GRAPH:
            try:
                graph = flight_graph("Rocket launch", 800, 500, 5)

                sub_data_0 = [{'x': time,
                               'y': optimal_velocity_log,
                               'color': "#008888",
                               'label': "Optimum  velocity"},
                              {'x': time,
                               'y': velocity_log,
                               'color': "#0000ff",
                               'label': "Current velocity"}]
                graph.add_plot("Velocity (km/s)", sub_data_0)

                sub_data_1 = [{'x': (0, self.time_final),
                               'y': (1, 1),
                               'color': "#880088",
                               'label': "Maximum thrust"},
                              {'x': time,
                               'y': throttle_set,
                               'color': "#ff0000",
                               'label': "Current throttle"}]
                graph.add_plot("Throttle (%)", sub_data_1)

                sub_data_2 = [{'x': time,
                               'y': height,
                               'color': "#008800",
                               'label': "Current height"}]
                graph.add_plot("Height (km)", sub_data_2)

                sub_data_3 = [{'x': time,
                               'y': fuel_level,
                               'color': "#888800",
                               'label': "Current fuel"}]
                graph.add_plot("Fuel (kg)", sub_data_3)

                sub_data_4 = [{'x': time,
                               'y': self.thrust,
                               'color': "#0000ff",
                               'label': "Thrust"},
                              {'x': time,
                               'y': self.gravity,
                               'color': "#008800",
                               'label': "Gravity"},
                              {'x': time,
                               'y': self.drag,
                               'color': "#ff0000",
                               'label': "Drag"}]
                graph.add_plot("Force (N)", sub_data_4)

                graph.done()

            except Exception as exp:
                output += 'Error plotting results:' + str(exp)
                output += '\n'
                import traceback
                print(traceback.format_exc())

        # Score for following optimal course
        scored_time_steps = 0
        scored_height_steps = 0

        student_velocity_score = 0
        student_height_score = 0

        total_time = flight_plan.get_plan_length()

        for time_step in range(0, total_time + 1):
            (expected_velocity, allowed_velocity_variance,
             expected_height, allowed_height_variance) = flight_plan.get_current_values(time_step)

            if allowed_velocity_variance is not None:
                scored_time_steps += 1

                lower_velocity = expected_velocity - allowed_velocity_variance
                upper_velocity = expected_velocity + allowed_velocity_variance

                if lower_velocity <= velocity_log[time_step] <= upper_velocity:
                    student_velocity_score += 1
                else:
                    print(f'Velocity not met at time step: {time_step} ::: ' +
                          f'{lower_velocity} <= {velocity_log[time_step]} <= {upper_velocity}')

            if allowed_height_variance is not None:
                scored_height_steps += 1

                lower_height = expected_height - allowed_height_variance
                upper_height = expected_height + allowed_height_variance

                if lower_height <= height[time_step] <= upper_height:
                    student_height_score += 1
                else:
                    print(f'Height not met at time step: {time_step} ::: ' +
                          f'{lower_height} <= {height[time_step]} <= {upper_height}')

        student_score = student_velocity_score + student_height_score
        expected_score = scored_time_steps + scored_height_steps
        flight_score = student_score / expected_score * ROCKET_FLIGHT_SCORE

        # Score for making a successful landing
        if good_landing:
            landing_score = ROCKET_LAND_SCORE
        else:
            landing_score = 0

        rocket_score = min(flight_score + landing_score, ROCKET_FLIGHT_SCORE + ROCKET_LAND_SCORE)

        return rocket_score, landing_score, flight_score, output


class PIDTestResult(unittest.TestResult):

    def __init__(self, stream=None, descriptions=None, verbosity=None):
        super(PIDTestResult, self).__init__(stream, verbosity, descriptions)
        self.stream = stream
        self.credit = []

    def stopTest(self, test):
        super(PIDTestResult, self).stopTest(test)
        try:
            self.credit.append(test.last_credit)

        except AttributeError as exp:
            self.stream.write(str(exp))

    @property
    def avg_credit(self):
        try:
            return sum(self.credit) / len(self.credit)

        except ZeroDivisionError:
            return 0.0


class PressurePDTestCase(unittest.TestCase):
    credit = []

    def setUp(self):
        """
        Init test setup.
        """
        self.last_result = ''
        self.last_credit = 0.0

        if studentExc:
            self.last_result = str(studentExc)
            raise studentExc

    def run_with_params(self, params: Dict):
        """
        Run test case using desired parameters.

        Args:
            params: a dictionary of test parameters.
        """
        target_pressure = params.get('target_pressure')
        score, output = PressurePD().start_launch(target_pressure, student_pressure_pd_solution)
        self.last_credit = score * PRESSURE_PERCENT
        self.last_result = (f'Pressure PD test case {params.get("test_case")}\n' +
                            f'{output}\n' +
                            f'Case score: {score}%\n' +
                            f'--------------------------------------------------------')

        print(self.last_result)

        self.assertTrue(score >= 100, 'Pressure PD submission did not correctly maintain pressure.')

    def test_case1(self):
        params = {'test_case': 1, 'target_pressure': TestCases.PressureTestCases.test_case1()}
        self.run_with_params(params)

    def test_case2(self):
        params = {'test_case': 2, 'target_pressure': TestCases.PressureTestCases.test_case2()}
        self.run_with_params(params)

    def test_case3(self):
        params = {'test_case': 3, 'target_pressure': TestCases.PressureTestCases.test_case3()}
        self.run_with_params(params)

    def test_case4(self):
        params = {'test_case': 4, 'target_pressure': TestCases.PressureTestCases.test_case4()}
        self.run_with_params(params)

    def test_case5(self):
        params = {'test_case': 5, 'target_pressure': TestCases.PressureTestCases.test_case5()}
        self.run_with_params(params)


class RocketPIDTestCase(unittest.TestCase):
    credit = []

    def setUp(self):
        """
        Init test setup.
        """
        self.last_result = ''
        self.last_credit = 0.0

        if studentExc:
            self.last_result = str(studentExc)
            raise studentExc

    def run_with_params(self, params: Dict):
        """
        Run test case using desired parameters.

        Args:
            params: a dictionary of test parameters.
        """
        flight_plan = params.get('flight_plan')
        fuel = params.get('fuel')
        (score, landing_score, flight_score,
         output) = RocketPID().launch_rocket(flight_plan, fuel, student_rocket_pid_solution)
        self.last_credit = score * ROCKET_PERCENT
        self.last_result = (f'Rocket PID test case {params.get("test_case")}\n' +
                            f'{output}\n' +
                            f'Optimal flight score: {flight_score}\n' +
                            f'Landing score: {landing_score}\n' +
                            f'Case score: {score}%\n' +
                            f'--------------------------------------------------------')

        print(self.last_result)

        self.assertTrue(landing_score == ROCKET_LAND_SCORE, 'Rocket did not land successfully.')
        self.assertTrue(flight_score >= ROCKET_FLIGHT_SCORE, 'Rocket flight did not follow optimal flight path.')

    def test_case1(self):
        flight_plan, fuel = TestCases.RocketTestCases.test_case1()
        params = {'test_case': 1, 'flight_plan': flight_plan, 'fuel': fuel}
        self.run_with_params(params)


def run_all(stream):
    suites = map(lambda case: unittest.TestSuite(unittest.TestLoader().loadTestsFromTestCase(case)),
                 [PressurePDTestCase, RocketPIDTestCase])

    average_scores = []
    for suite in suites:
        result = PIDTestResult(stream=stream)
        suite.run(result)
        average_scores.append(result.avg_credit)

    stream.write('\nFinal Scores\n\n')
    stream.write(f'Pressure PD score: {average_scores[0]}\n')
    stream.write(f'Rocket PID score: {average_scores[1]}\n')
    stream.write(f'\n')

    total_score = round(sum(average_scores))
    stream.write(f'score: {total_score}\n')


if __name__ == "__main__":
    student_id = who_am_i()
    if student_id:
        try:
            # parser = argparse.ArgumentParser()
            # parser.add_argument('-s', '--showgraph', type=str, default=None, nargs='?', const='default',
            #                     help="Select visualization: 'turtle', 'matplotlib'")
            # args = parser.parse_args()
            # showgraph = args.showgraph or ""

            showgraph = 'matplotlib'

            if showgraph.lower() in ['turtle', 'matplotlib', 'default']:
                SHOW_GRAPH = True

                # attempt to import graphing visualizer
                if showgraph.lower() == 'turtle':
                    from FlightData.TurtleVisualizer import TurtleVisualizer

                    flight_graph = TurtleVisualizer
                elif showgraph.lower() == 'matplotlib':
                    try:
                        from FlightData.MatPlotLibVisualizer import MatPlotLibVisualizer

                        flight_graph = MatPlotLibVisualizer
                    except ImportError:
                        raise ValueError(
                            "MatPlotLib not installed. Please install module or use 'turtle' visualizer argument.")

            else:
                SHOW_GRAPH = False

                print("Disabling visualization.  Use --showgraph option to enable.")

            run_all(sys.stdout)
        except Exception as e:
            print(e)
            print('score: 0')
    else:
       print("Student ID not specified.  Please fill in 'whoami' variable.")
       print('score: 0')
