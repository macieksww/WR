#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
from sensor_msgs.msg import LaserScan
from math import pi
from math import atan2
from math import sqrt
from math import tan



goal = (13, 1.5) # krotka przechowująca współrzędne celu - domyślnie 13, 1.5 


state = 0 # stan w którym znajduje się agent - domyślnie 0
left_side_turn_left_flag = 0
right_side_turn_right_flag = 0
min_distance_to_goal = 1000 # minimalna odległość do celu - domyślnie ustawiona na bardzo dużą
drive_to_finish_flag = 0
previous_state = 0 # stan poprzedni domyślnie ustawiony na 0
list_of_touch_points = [] # lista punktów dojazdu do ściany
list_of_ranges = [] # lista w której zapisane zostaną odległości, które zwraca miernik ze wszystkich kątów
angle_to_goal = 0 # kąt do celu
already_touched_flag = 0 # flaga, która przyjmie wartość 1 w momencie, gdy punkt w którym znajduje się robot jest jednym z "hitpointów" (jest w liście list_of_touch_points)
already_touched_start_theta_position = 0 # zmienna przyjmie wartość pozycji kątowej robota przed obrotem (po dojeździe do hitpointa), w ALG2 i REV2 (stan 6)
end_theta = 0 # wartość wyjściowej pozycji kątowej agenta w stanie 6 (already_touched_start_theta_position +/- pi). Jeżeli robot osiągnie pozycję kątową ent_theta to stwierdzam, że się obrócił
right_side_turn_right_flag = 0
exit_change_side_flag = 0
number_of_touch_points = 0 # liczba hitpointów
start_side = "right" # strona, którą robot będzie na począku jechał przy ścianie - domyślnie prawa
current_side = "right" # aktualna strona jazdy przy ścianie - domyślnie prawa
side_before_touch = "right" # stona przed dojazdem do następnej ściany - używana w algorytmie REV 2 aby określić następną stronę, którą robot będzie jechał przy ścianie - domyślnie prawa
change_side_flag = 0
previous_state_ALG2 = 0
list_of_visited_points = [] # jeżeli robot dojedzie do jakiegoś hitpointa (ALG2/ REV2) i zacznie się obracać, to punkt ten zostaje dodany do list list_of_visited_points
algorithm_choice = "ALG" # wybór algorytmu - domyślnie ALG
list_of_depart_points_BUG2 = [] # lista punktów odjazdu od ściany (przejścia ze stanu jazda przy ścianie do stanu jazda w kierunku celu w algorytmie BUG2)
num_in_list = 0 # index w liście elementu o najmniejsze wartości (najmniejszej odległości)



# funkcja wywolywana przy przyjsciu danych ze skanera laserowego

def choose_direction (directions):

	num_in_list = 0
	global angle
	global dir # minimalna odległość do celu
	global left_side_front # odległość z kąta 70 stopni
	global left_side_fronter # odległość z kąta 45 stopni
	global left_side_back # odległość z kąta 110 stopni
	global right_side_front # odległość z kąta 290 stopni
	global right_side_fronter # odległość z kąta 315 stopni
	global right_side_back # odległość z kąta 250 stopni
	global left_side_perpendicular # odległość z kąta 90 stopni
	global right_side_perpendicular # odległość z kąta 270 stopni
	global head_min # minimalna odległość z "wycinka czołowego" - od 350 do 10 stopni
	global head # odległość z kąta 0 stopni
	global direction_10 # odległość z kąta 10 stopni
	global direction_350 # odległość z kąta 350 stopni
	global left_side_min_quarter # minimalna odległość z wycinka od 0 do 100 stopni
	global right_side_min_quarter # minimalna odległość z wycinka od 260 do 359 stopni
	global list_of_ranges # listawszystkich odległości zwracanych przez miernik 
	global num_in_list # index w liście elementu o najmniejsze wartości (najmniejszej odległości)


	#
	# Ustawianie wartości wymienionych wyżej zmiennych globalnych
	#

	head_min = min(min(directions[0:10]), min(directions[350:359]))
	direction_10 = directions[10]
	direction_350 = directions[350]
	dir = min(directions)
	for direction in directions:
		if (direction == dir):
			print "INDEX", num_in_list
			angle = num_in_list*2*pi/360
			print "LOOP ANGLE", angle
			break
		else:
			num_in_list += 1

	head = directions[0]

	left_side_front = directions[70] 
	left_side_fronter = directions[45]
	left_side_back = directions[110]
	left_side_perpendicular = directions[90]
	left_side_min_quarter = min(directions[0:100])
	list_of_ranges = directions[0:359]


	right_side_front = directions[290]
	right_side_fronter = directions[315]
	right_side_back = directions[250]
	right_side_perpendicular = directions[270]
	right_side_min_quarter = min(directions[260:359])

	return (dir, angle, left_side_front, left_side_back , left_side_fronter) 

### FUNKCJE WYBORY ALGORYTMU PRZECHODZENIA TRASY

# Funkcja BUG2 definiuje przechodzenie między funkcjami i zmieny stanów dla algorytmu alg2


def BUG2 (position_x, position_y, position_theta):
	global drive_to_finish_flag
	global min_distance_to_goal
	# SŁOWNIK STANÓW:
	# 0 - Stan początkowy programu
	# 1 - Stan w którym robot jedzie w kierunku celu
	# 3 - Stan definiujący jazdę robota przy ścianie
	# 5 - Stan definiujący skręt wewnętrzny

	# Sprawdzenie, czy robot nie znajduje się na linii łączącej punkt startowy z celem

	if check_if_on_line(position_x, position_y) == True: # sprawdzenie, czy jest na linii
		if check_if_not_same_point(position_x, position_y) == True: # sprawdzenie, czy nie znajduje się w punkcie, z którego już w przeszłości odjechal od ściany 
			temp_distance_to_goal = count_distance_to_goal(position_x, position_y) # odległość do celu z aktualnego punktu (musi być mniejsza od dotychczas najmniejszej, aby robot odjechał od ściany w kierunku celu)
			if check_if_wall_not_on_the_line(position_x, position_y, position_theta)== True and state != 1 and temp_distance_to_goal < min_distance_to_goal: # jeżeli spełnione są powyższe warunki
				change_min_distance_to_goal(temp_distance_to_goal) # zmiana minimalnej odległości od ściany na akutalną
				list_of_depart_points_BUG2.append((position_x, position_y)) # dodaję punkt, w którym robot odjechał od ściany do listy punktów odjazdu
				change_state() # wywołanie funkcji zmieniającej stan z 3 na 1 (jazda przy ścianie -> jazda w kierunku celu)


	if state == 0 or state == 1:
		if current_side == "right":
			drive_to_finish_right(position_theta, position_x, position_y)
		else:
			drive_to_finish (position_theta, position_x, position_y)
	elif state == 3:
		drive_to_finish_flag = 0
		if current_side == "right":
			right_side_drive_next_wall(position_theta)
		else:
			left_side_drive_next_wall(position_theta)
	elif state == 5:
		if current_side == "right":
			right_side_turn_left(position_theta)
		else:
			left_side_turn_right(position_theta)


# Funkcja ALG2 definiuje przechodzenie między funkcjami i zmieny stanów dla algorytmu alg2


def ALG2 (position_x, position_y, position_theta, directions):
	global current_side
	global start_side
	global previous_state_ALG2
	global drive_to_finish_flag
	change_goal_distance()

	# SŁOWNIK STANÓW:
	# 0 - Stan początkowy programu
	# 1 - Stan w którym robot jedzie w kierunku celu
	# 3 - Stan definiujący jazdę robota przy ścianie
	# 5 - Stan definiujący skręt wewnętrzny
	# 6 - Stan definiujący obrót robota po dojechaniu do punktu styku ze ścianą (jeden z poprzednich punktów, w których robot dojeżdza do ściany i zaczyna jazdę przy niej)

	if state == 0 or state == 1:
		drive_to_finish_right_ALG2(position_theta, position_x, position_y)

	elif state == 3:
		drive_to_finish_flag = 0
		if current_side == "right":
			right_side_drive_next_wall_ALG2(position_x, position_y, position_theta, directions)
		else:
			left_side_drive_next_wall_ALG2(position_x, position_y, position_theta, directions)
	elif state == 5:
		if current_side == "right":
			right_side_turn_left(position_theta)
		else:
			left_side_turn_right(position_theta)
	elif state == 6:
		change_side_if_touched(position_theta)


# Funkcja REV2 definiuje przechodzenie między funkcjami i zmieny stanów dla algorytmu alg2



def REV2 (position_x, position_y, position_theta, directions):
	global current_side
	global change_side_flag
	global start_side
	global previous_state_ALG2
	global drive_to_finish_flag
	global side_before_touch
	change_goal_distance()

	# SŁOWNIK STANÓW:
	# 0 - Stan początkowy programu
	# 1 - Stan w którym robot jedzie w kierunku celu
	# 3 - Stan definiujący jazdę robota przy ścianie
	# 5 - Stan definiujący skręt wewnętrzny
	# 6 - Stan definiujący obrót robota po dojechaniu do punktu styku ze ścianą (jeden z poprzednich punktów, w których robot dojeżdza do ściany i zaczyna jazdę przy niej)



	if state == 0 or state == 1:
		if current_side == "right":
			drive_to_finish_right_REV2(position_theta, position_x, position_y)
		else:
			drive_to_finish_REV2(position_theta, position_x, position_y)
	elif state == 3:
		if current_side == "right":
			right_side_drive_next_wall_ALG2(position_x, position_y, position_theta, directions)
			side_before_touch = "right"
		else:
			left_side_drive_next_wall_ALG2(position_x, position_y, position_theta, directions)
			side_before_touch = "left"
	elif state == 5:
		if current_side == "right":
			right_side_turn_left(position_theta)
		else:
			left_side_turn_right(position_theta)
	elif state == 6:
		change_side_if_touched(position_theta)



### FUNKCJE UZYWANE PRZY WSZYSTKICH ALGORYTMACH

# funckja wywołująca zwrot z miernika odległości

def scan_callback(scan):
	choose_direction(scan.ranges)

#funkcja zwaraca informacje, czy agent dotarł do końca labirytnu
#jeśli tak, to zatrzymuje robota i zwraca informacje o opuszczeniu labiryntu

def check_if_in_finish(position_x, position_y):
	if distance_from_2_points(goal[0], goal[1], position_x, position_y) < 0.35: # sprawdzenie, czy odległość z aktualnego punktu do celu jest mniejsza niż 0.35. JEśli tak - robot jest u celu
		new_vel.linear.x = 0
		new_vel.angular.z = 0
		print ("SUKCES, ROBOT OPUSCIL LABIRYNT")
		return True
	else:
		return False


#funkcja jest używana przy skrętach "zamkniętych"
#funkcja zwraca wartość True, gdy wykryje zakęt - odległość od kąta 0 względem czoła robota < 0.6 metra

def check_head():
	if head > 0.6: # jeżeli odległość z kąta 0 > 0.6
		return False
	else:
		 return True

#funkcja zwraca wartosc kata miedzy dwoma punktami

def count_dest_angle(c2y, c1y, c2x, c1x):
	global dest_angle
	dest_angle = atan2 (c2y-c1y, c2x-c1x)
	return dest_angle

#funkcja zwraca aktualną odległość agenta do celu
#wykorzystywana w algorytmie BUG2

def count_distance_to_goal(position_x, position_y):
	global distance_to_goal
	distance_to_goal = sqrt((goal[1] - position_y)**2 + (goal[0] - position_x)**2)
	return distance_to_goal

# funkcja drive_to_finish wykorzystywana w algorytmie BUG2 
# funkcja jest wykonywana w stanie 1 i służy do jazdy do celu podczas,
# gdy robot okrąża labirynt lewą stroną do ściany








# FUNKCJE UŻYWANE PRZY ALGORYTMIE BUG2






# funckja opisuje zachowanie robota, gdy ten posiada wolną przestrzeń w kierunku celu - jazda do celu
# oraz skierowany jest lewą stroną do ściany


def drive_to_finish (position_theta, position_x, position_y):
	global state
	global theta_before_right_turn
	global drive_to_finish_flag
	global angle_to_goal
	
    # global list_of_touch_points
	angle_to_goal = check_goal_accessibility(position_x, position_y)
	
	print left_side_min_quarter, "MIN_QUARTER"

	if abs(position_theta - angle_to_goal) > 0.1 and (left_side_min_quarter > 0.7 or drive_to_finish_flag == 0): # jeżeli pozycja theta nie jest równa kątowi do celu, to robot powinien się obrócić
		if position_theta <= angle_to_goal: # nadaję prędkość kątową z zależną od pozycji katowej robota 0.3 / -0.3
			new_vel.angular.z = 0.3
		elif position_theta > angle_to_goal:
			new_vel.angular.z = -0.3
		new_vel.linear.x = 0
		state = 1
		print ("Rotating to goal angle")
	else: # jeżeli robot zkieruje się w kierunku celu
		state = 1
		drive_to_finish_flag = 1
		#print ("Following goal angle")

		if  left_side_min_quarter > 0.5: # jeżeli droga jest wolna - robot porusza się z prędkościa liniową x
			new_vel.angular.z = 0
			new_vel.linear.x = 0.3
			print ("GO AHEAD")

		else: # jeżeli drpga nie jest wolna

			if (abs(left_side_front - left_side_back) > 0.05 or left_side_perpendicular > 0.5): # robot obraca się w kierunku ściany
				new_vel.angular.z = -0.3
				new_vel.linear.x = 0
				print ("ROTATE TO WALL FOLLOW")
			else:
				new_vel.linear.x = 0
				state = 3 # jeżeli robot się obróci to stan zmienia się na jazdę przy ścianie

# funkcja drive_to_finish_right wykorzystywana w algorytmie ALG2 
# funkcja jest wykonywana w stanie 1 i służy do jazdy do celu podczas,
# gdy robot okrąża labirynt prawą stroną do ściany

def drive_to_finish_right (position_theta, position_x, position_y):
	global state
	global theta_before_right_turn
	global drive_to_finish_flag
	global angle_to_goal
	global number_of_touch_points

    # global list_of_touch_points
	angle_to_goal = check_goal_accessibility(position_x, position_y)
	
	print left_side_min_quarter, "MIN_QUARTER"

	if abs(position_theta - angle_to_goal) > 0.1 and (right_side_min_quarter > 0.7 or drive_to_finish_flag == 0):
		if position_theta <= angle_to_goal:
			new_vel.angular.z = 0.3
		elif position_theta > angle_to_goal:
			new_vel.angular.z = -0.3
		new_vel.linear.x = 0
		state = 1
		print ("Rotating to goal angle")
	else:
		state = 1
		drive_to_finish_flag = 1
		print ("DISTOWALL", dir)
		print ("FRONT", right_side_front)
		print ("BACK", right_side_back)
		if  right_side_min_quarter > 0.5:
			new_vel.angular.z = 0
			new_vel.linear.x = 0.3
			print ("GO AHEAD")

		else:

			if (abs(right_side_back - right_side_front) > 0.05 or right_side_perpendicular > 0.5):
				new_vel.angular.z = 0.3
				new_vel.linear.x = 0
				print ("ROTATE TO WALL FOLLOW")
			else:
				new_vel.linear.x = 0
				temp_distance_to_goal = count_distance_to_goal(position_x, position_y)
				change_min_distance_to_goal(temp_distance_to_goal)
				state = 3


# funkcja left_side_drive_next_wall jest wykonywana w algorytmie BUG2 
# podczas jazdy robota przy ścianie lewą stroną

def left_side_drive_next_wall(position_theta):
	global state
	global theta_before_right_turn
	global angle_type
	global end_theta
	global start_side
	global exit_change_side_flag
	global current_side
	global list_of_visited_points
	global open_turn_flag
	global num_in_list
	get_closer_flag = 0
	get_farer_flag = 0
	out_of_band_flag = 0
	too_close_flag = 0
	open_turn_flag = 0


	print ("STAN 3")
	if check_head() == False:
		if dir > 0.3 and dir < 0.5: # jeżeli robot znajduje się w pasie
			open_turn_flag = 0
			if (dir > 0.4 and dir < 0.5) and out_of_band_flag == 1:
				new_vel.angular.z = -0.2 * ((dir - 0.4)/0.2) # jeżeli robot znajduje się w dalszej częsci pasa to wjeżdża w niego dostosowując swoją prędkosć do odległości od ściany
				new_vel.linear.x = 0.25
				too_close_flag = 0

			elif (dir > 0.3 and dir < 0.4) and too_close_flag == 1:
				new_vel.angular.z = 0.2 * ((0.4 - dir)/0.2) # jeżeli robot znajduje się w bliższej częsci pasa to wjeżdża w niego dostosowując swoją prędkosć do odległości od ściany
				new_vel.linear.x = 0.25
				out_of_band_flag = 0
			
			if (round(left_side_front, 2) == round(left_side_back, 2)): # jeżeli odległości left_side_front i left_side_back są równe to  robot posiada tylko prędkość liniową x
				new_vel.linear.x = 0.4
				new_vel.angular.z = 0
				print ("LINE FOLLOW")
			else:
				if left_side_front >= left_side_back: # jeżeli odległości left_side_front > left_side_back to robot równa pozycję tak, aby być ustawiony równolegle do ściany
					new_vel.angular.z = 0.2
					new_vel.linear.x = 0.2
					print ("EQUALISE FROM FRONT TO BACK")
				else:
					new_vel.angular.z = -0.2  # jeżeli odległości left_side_front < left_side_back to robot równa pozycję tak, aby być ustawiony równolegle do ściany
					new_vel.linear.x = 0.2
					print ("EQUALISE FROM BACK TO FRONT")
				return state

		elif dir > 0.5: # jeżeli robot wyjedzie poza pas
			open_turn_flag = 1
			new_vel.linear.x = 0
			if num_in_list > 60 and num_in_list < 80: # jeżeli minimalna odległość pochodzi z kątów (60 - 80) to robot jedzie w stronę sciany
				new_vel.angular.z = 0
				new_vel.linear.x = 0.2
				print ("APPROACH DRIVE BAND")
			else:  # jeżeli minimalna odległość niepochodzi z kątów (60 - 80) to robot dokręca w stronę sciany
				new_vel.linear.x = 0
				new_vel.angular.z = 0.2
				print ("ROTATING TO WALL")


		elif dir < 0.3: # jeżeli robot wyjedzie z pasa w strone bliższą ściany to to wjeżdża w niego dostosowując swoją prędkosć do odległości od ściany 
			new_vel.angular.z = -0.2*((0.3 - dir)/0.2)
			new_vel.linear.x = 0.3
	
	elif check_head() == True: # sprawdzenie czy robot nie wykrył skrętu zamkniętego
		new_vel.angular.z = 0
		new_vel.linear.x = 0
		state = 5 # jeśli tak, to następuje zmiana stanu na stan 5
		theta_before_right_turn = position_theta
		print ("RIGHT TURN DETECTED")
if exit_change_side_flag > 0:
	exit_change_side_flag -= 1



# funkcja right_side_drive_next_wall jest wykonywana w algorytmie BUG2 
# podczas jazdy robota przy ścianie prawą stroną


def right_side_drive_next_wall(position_theta):
	global state
	global theta_before_left_turn
	global angle_type
	global open_turn_flag
	global num_in_list
	get_closer_flag = 0
	get_farer_flag = 0
	out_of_band_flag = 0
	too_close_flag = 0
	open_turn_flag = 0


	print ("STAN 3")
	if check_head() == False :

		print "DIST TO WALL", dir
		print ("NUM IN LIST", num_in_list)

		if dir > 0.3 and dir < 0.5:
			open_turn_flag = 0
			if dir > 0.4 and dir < 0.5 and out_of_band_flag == 1:
				new_vel.angular.z = 0.2 * ((dir - 0.4)/0.2)
				new_vel.linear.x = 0.25

			elif dir > 0.3 and dir < 0.4 and too_close_flag == 1:
				new_vel.angular.z = -0.2 * ((0.4 - dir)/0.2)
				new_vel.linear.x = 0.25

			if (round(right_side_front, 2) == round(right_side_back, 2)):
				new_vel.linear.x = 0.4
				new_vel.angular.z = 0
				print ("LINE FOLLOW")
			else:
				if get_closer_flag == 0 and get_farer_flag == 0:
					if right_side_front >= right_side_back:
						new_vel.angular.z = -0.2
						new_vel.linear.x = 0.2
						print ("EQUALISE FROM FRONT TO BACK")
					else:
						new_vel.angular.z = 0.2
						new_vel.linear.x = 0.2
						print ("EQUALISE FROM BACK TO FRONT")
					return state


		elif dir > 0.5:
			open_turn_flag = 1
			new_vel.linear.x = 0
			if num_in_list > 280 and num_in_list < 300:
				new_vel.angular.z = 0
				new_vel.linear.x = 0.2
				print ("ROTATING TO WALL")
			else:
				new_vel.linear.x = 0
				new_vel.angular.z = -0.2
				print ("APPROACH DRIVE BAND")


		elif dir < 0.3:
			new_vel.angular.z = 0.2*((0.3-dir)/0.2)
			new_vel.linear.x = 0.3

	
	elif check_head() == True:
		new_vel.angular.z = 0
		new_vel.linear.x = 0
		state = 5
		theta_before_left_turn = position_theta
		print ("RIGHT TURN DETECTED")


# funkcja wykorzystywana jest w algorytmach BUG2, ALG2, REV2
# funkcja dziefinuje skret zamknięty w prawo jaki wykonuje robot jadąc lewą stroną przy ścianie

	
def left_side_turn_right(position_theta):
	global state
	global theta_before_right_turn
	global theta_in_acute_angle_turn
	global right_turn_flag
	global previous_state_ALG2
	global head
	global num_in_list

	if head < 2: # jeżeli spełniony jest warunek że odległość z kąta 0 < 2
		if num_in_list > 83 and num_in_list < 97: # jeżeli najmniejsza odległość zwrócona przez miernik jest w zakresie kątów (83 - 97)
			new_vel.linear.x = 0.3 # robot jedzie do przodu
			new_vel.angular.z = 0
		else:
			new_vel.linear.x = 0
			new_vel.angular.z = -0.2 # robot "dokręca" w prawo

		if num_in_list > 83 and num_in_list < 97:# jeżeli najmniejsza odległość zwrócona przez miernik  jest w zakresie kątów (83 - 97)

			if dir > 0.4: # dokonuję regulacji pozycji robota względem ściany - tak aby osiągnąć możniwie równoległą pozycję wobec niej
				new_vel.linear.x = 0.15
				new_vel.angular.z = 0.4
				print ("GET CLOSER TO THE WALL")

			if dir < 0.3:
				new_vel.linear.x = 0.15
				new_vel.angular.z = -0.3
				print ("GET FARER FROM THE WALL")
	else:
		state = 3 # jeżeli odległość z kąta 0 będzie > 2 to robot przechodzi do stanu 3 - jazda przy ścianie


# funkcja wykorzystywana jest w algorytmie BUG2 
# funkcja dziefinuje skret zamknięty w lewo jaki wykonuje robot jadąc prawą stroną przy ścianie



def right_side_turn_left(position_theta):
	global state
	global theta_before_left_turn
	global theta_in_acute_angle_turn
	global left_turn_flag
	global previous_state_ALG2
	global head
	global num_in_list

	if head < 2:
		if num_in_list > 263 and num_in_list < 277:
			new_vel.linear.x = 0.3
			new_vel.angular.z = 0
		else:
			new_vel.linear.x = 0
			new_vel.angular.z = 0.2

		if num_in_list > 263 and num_in_list < 277:

			if dir > 0.4:
				new_vel.linear.x = 0.15
				new_vel.angular.z = -0.3
				print ("GET CLOSER TO THE WALL")

			if dir < 0.3:
				new_vel.linear.x = 0.15
				new_vel.angular.z = 0.3
				print ("GET FARER FROM THE WALL")
	else:
		state = 3



#
# FUNKCJE SŁUŻĄCE DO WYZNACZENIA PARAMERTÓW PROSTEJ W BUG2 I SPRAWDZENIA MOŻLIWOŚCI WŁĄCZENIA SIĘ STANU 1 (JAZDA DO CELU) W ALGORYTMIE BUG2
#


# funkcja count_line_parameters jest wykorzystywana przy algorytmie BUG2
# służy do wyznaczenia współczynnika kierunkowego a i wyrazu wolnego b 
# prostej łączącej punkt startowy robota z wyjście z labiryntu


def count_line_parameters():
	global a
	global b
	a = tan(goal[1] - 0.5)/(goal[0] - 0.5)
	b = goal[1] - a*goal[0]
	return (a, b)


# funkca check_goal_accessibility wyznacza kat pomiędzy aktualną pozycją agenta a wyjściem z labiryntu

def check_goal_accessibility(position_x, position_y):
	position_goal_angle = (atan2(goal[1]-position_y, goal[0]-position_x)) # goal[1] współrzędna y cel, goal[0] - współrzędna x celu
	return position_goal_angle

# funkcja check_if_on_line służy do sprawdzenia, czy robot znajduje się na prostej łaczącej punkt startowy z wyjściem z labiryntu BUG2

def check_if_on_line(position_x, position_y):
	parameters = count_line_parameters() # wyliczenie parametrów prostej łączącej punkt startowy z celem
	parameter_a = parameters[0]
	parameter_b = parameters[1]

	if (abs((position_x * parameter_a + parameter_b) - position_y) < 0.3): # jeżeli punkt znajduje się na prostej funkcja zwraca True, jeśl nie zwraca False
		return True
	else:
		return False

# funkcja check_if_wall_not_on_the_line służy do sprawdzenia, czy na drodze do celu (w stanie 1) nie stoi przeszkoda

def check_if_wall_not_on_the_line(position_x, position_y, position_theta):
	global current_side
	angle_to_finish = atan2(goal[1] - position_y, goal[0] - position_x) # wyliczam kąt do celu

	if current_side == "left":
		if position_theta >= 0 and position_theta <= pi: # jeżeli pozycja theta jest z przedziału (0, pi)
			if (angle_to_finish >= position_theta or angle_to_finish >= -1*pi and angle_to_finish <= position_theta - pi): # jeżeli kąt do celu nie mieści się w określonym przedziale to funkcja zwraca False
				return False
			else:
				return True

		elif position_theta >= -1*pi and position_theta < 0: # jeżeli pozycja theta jest z przedziału (-pi, 0)
			if (angle_to_finish > position_theta and angle_to_finish < position_theta + pi):  # jeżeli kąt do celu nie mieści się w określonym przedziale to funkcja zwraca False
				return False
			else:
				return True

	elif current_side == "right":
		if position_theta >= 0 and position_theta <= pi: # jeżeli pozycja theta jest z przedziału (0, pi)
			if (angle_to_finish >= position_theta or angle_to_finish >= -1*pi and angle_to_finish <= position_theta - pi): # jeżeli kąt do celu nie mieści się w określonym przedziale to funkcja zwraca False
				return True
			else:
				return False

		elif position_theta >= -1*pi and position_theta < 0: # jeżeli pozycja theta jest z przedziału (-pi, 0)
			if (angle_to_finish > position_theta and angle_to_finish < position_theta + pi): # jeżeli kąt do celu nie mieści się w określonym przedziale to funkcja zwraca False
				return True
			else:
				return False

# funkcja służy do sprawdzenia, czy robot nie odwiedził już danego punktu i nie przełączył się w nim na stan 1
# funkcja jest konieczna do zaimplementowania, ponieważ warunek mówiący o tym, że robot ma jechać do celu (włączać stan 1)
# nie jest wystarczający w przypadku dwukrotnego najechania na prostą w prawie tym  samym punkcie
# podczas drugiego najechania robot może złapać kontak z linią w niewiele mniejszej niż za pierwszym razm odległości od celu
# (np kilka centymetrów) i wywoła to przełączenie na stan 1 i jazdę do celu. Aby tego uniknąć funkcja check_if_not_same_point
# sprawdza, czy punkt (lub blisko znajdujący się punkt) w którym robot ma stycznosć z linią nie był już odwiedzony
# jeśli był, to agent nie przełącza swojego stanu na stan 1 (dojazd do celu), tylko cały czas kontynuuje jazdę przy ścianie

def check_if_not_same_point(position_x, position_y):
	global list_of_depart_points_BUG2
	min_distance = 100
	for point in list_of_depart_points_BUG2:
		temp_points_distance = distance_from_2_points(point[0], point[1], position_x, position_y)
		if temp_points_distance < min_distance:
			min_distance = temp_points_distance
	if min_distance < 0.5: # jeżeli najbliższa odległość od punktu odjazdu < 0.5 to funkcja zwraca False - oznacza to, że robot dojechał do punkt, z którego już raz odjechał od ściany
		return False # nie powinien więc odjechać drugi raz (przejść do stanu jazdy w kierunku celu)
	else:
		return True # jeżeli najbliższa odległość od punktu odjazdu > 0.5 to funkcja zwraca True - oznacza to, że robot dojechał do nie punktu, z którego już raz odjechał od ściany
		# więc może odjeżdzać od ściany (przejść do stanu jazdy w kierunku celu)

# funkcja służy do zmiany stanu agenta ze stanu 3 na stan 1 w algorytmie BUG2

def change_state():
	global state
	global previous_state
	state = 1
	previous_state = 3
	return state, previous_state







# FUNKCJE UŻYWANE DO ALGORYTMU ALG2








# funkcja drive_to_finish_right_ALG2 wykorzystywana w algorytmie ALG2 
# funkcja jest wykonywana w stanie 1 i służy do jazdy do celu podczas,
# gdy robot okrąża labirynt lewą stroną do ściany


def drive_to_finish_right_ALG2 (position_theta, position_x, position_y):
	global state
	global theta_before_right_turn
	global drive_to_finish_flag
	global angle_to_goal
	global number_of_touch_points
	global change_side_flag
	global current_side
	global previous_state_ALG2

	
    # global list_of_touch_points
	angle_to_goal = check_goal_accessibility(position_x, position_y)
	
	print left_side_min_quarter, "MIN_QUARTER"

	if abs(position_theta - angle_to_goal) > 0.1 and (right_side_min_quarter > 0.7 or drive_to_finish_flag == 0):
		if position_theta <= angle_to_goal:
			new_vel.angular.z = 0.3
		elif position_theta > angle_to_goal:
			new_vel.angular.z = -0.3
		new_vel.linear.x = 0
		state = 1
		print ("Rotating to goal angle")
	else:
		state = 1
		drive_to_finish_flag = 1
		print ("DISTOWALL", dir)
		print ("FRONT", right_side_front)
		print ("BACK", right_side_back)
		if  right_side_min_quarter > 0.5:
			new_vel.angular.z = 0
			new_vel.linear.x = 0.3
			print ("GO AHEAD")

		else:
			if (abs(right_side_back - right_side_front) > 0.05 or right_side_perpendicular > 0.5):
				new_vel.angular.z = 0.3
				new_vel.linear.x = 0
				print ("ROTATE TO WALL FOLLOW")
			else:
				new_vel.linear.x = 0
				number_of_touch_points += 1
				list_of_touch_points.append((position_x, position_y))
				goal_distance = distance_from_2_points (goal[0], goal[1], position_x, position_y)
				change_min_distance_to_goal (goal_distance)
				previous_state_ALG2 = 1
				current_side = "right"
				state = 3



# funkcja left_side_drive_next_wall_ALG2 jest wykonywana w algorytmie ALG2 
# podczas jazdy robota przy ścianie lewą stroną

def left_side_drive_next_wall_ALG2(position_x, position_y, position_theta, directions):
	print ("STAN 3")
	global state
	global theta_before_right_turn
	global angle_type
	global already_touched_flag
	global already_touched_start_theta_position
	global end_theta
	global start_side
	global exit_change_side_flag
	global current_side
	global list_of_visited_points
	global open_turn_flag
	global num_in_list
	get_closer_flag = 0
	get_farer_flag = 0
	out_of_band_flag = 0
	too_close_flag = 0
	open_turn_flag = 0


	print ("FRONT", left_side_front)
	print ("BACK", left_side_back)


	current_angle_to_goal = check_goal_accessibility (position_x, position_y) # aktualny kąt do celu
	if check_if_obstacle_on_the_way_to_goal(position_x, position_y, position_theta, directions) == False: # jeżeli droga do celu jest wolna, to przełącza się stan robota na stan 1 - jazda w kierunku celu
		state = 1

	if state != 1: # jeżeli robot nie zmienił stanu na jazdę w kierunku celu (jest w stanie jazda przy ścianie)
		if already_touched_flag == 0: 
			if len(list_of_touch_points) > 1: # jeżeli ilość hitpointów > 1
				for iterator in range(0, len(list_of_touch_points)-1):
					distance_from_points = distance_from_2_points(list_of_touch_points[iterator][0], list_of_touch_points[iterator][1], position_x, position_y)
					if distance_from_points < 0.35 and number_of_touch_points > 1 and list_of_touch_points[iterator] not in list_of_visited_points:
						# jeżeli odległość od hitpointa < 0.35 i liczba hitpointów > 1 i aktualnie odwiedzany hitpoint nie został wcześniej odwiedzony
						already_touched_flag = 1 # ustawiam wartość flagi definiującej, czy robot ma zmienić stroną okrążania ściany na 1 - robot będzie okrążał ścianę
						end_theta = classify_touched_theta_position(position_theta) # wyznaczam wartość pozycji theta, dla której robot wyjdzie ze stanu 6 (obrót) do stanu 3 (jazda przy scianie)
						list_of_visited_points.append(list_of_touch_points[iterator]) # dodaję aktualny punkt do listy list_of_visited_points
						print ("Punkt juz odwiedzony")
						break
		if already_touched_flag == 1:
			start_side = "left"
			state = 6 # zmiana stanu na 6 - obrót

		if already_touched_flag == 0:
			if check_head() == False :
				print "DIST TO WALL", dir
				print ("NUM IN LIST", num_in_list)

				if dir > 0.3 and dir < 0.5:
					open_turn_flag = 0
					if (dir > 0.4 and dir < 0.5) and out_of_band_flag == 1:
						new_vel.angular.z = -0.2 * ((dir - 0.4)/0.2)
						new_vel.linear.x = 0.25
						too_close_flag = 0

					elif (dir > 0.3 and dir < 0.4) and too_close_flag == 1:
						new_vel.angular.z = 0.2 * ((0.4 - dir)/0.2)
						new_vel.linear.x = 0.25
						out_of_band_flag = 0
					
					if (round(left_side_front, 2) == round(left_side_back, 2)):
						new_vel.linear.x = 0.4
						new_vel.angular.z = 0
						print ("LINE FOLLOW")
					else:
						if left_side_front >= left_side_back:
							new_vel.angular.z = 0.2
							new_vel.linear.x = 0.2
							print ("EQUALISE FROM FRONT TO BACK")
						else:
							new_vel.angular.z = -0.2
							new_vel.linear.x = 0.2
							print ("EQUALISE FROM BACK TO FRONT")
						return state

				elif dir > 0.5:
					open_turn_flag = 1
					new_vel.linear.x = 0
					if num_in_list > 60 and num_in_list < 80:
						new_vel.angular.z = 0
						new_vel.linear.x = 0.2
						print ("ROTATING TO WALL")
					else:
						new_vel.linear.x = 0
						new_vel.angular.z = 0.2
						print ("APPROACH DRIVE BAND")


				elif dir < 0.3:
					new_vel.angular.z = -0.2*((0.3 - dir)/0.2)
					new_vel.linear.x = 0.3
					print "03"

			
			elif check_head() == True:
				new_vel.angular.z = 0
				new_vel.linear.x = 0
				state = 5
				theta_before_right_turn = position_theta
		if exit_change_side_flag > 0:
			exit_change_side_flag -= 1



# funkcja righ_side_drive_next_wall_ALG2 jest wykonywana w algorytmie ALG2 
# podczas jazdy robota przy ścianie prawą stroną



def right_side_drive_next_wall_ALG2(position_x, position_y, position_theta, directions):
	print ("STAN 3")
	global state
	global theta_before_left_turn
	global angle_type
	global already_touched_flag
	global already_touched_start_theta_position
	global end_theta
	global start_side
	global exit_change_side_flag
	global current_side
	global list_of_visited_points
	global num_in_list
	global side_before_touch
	global open_turn_flag
	get_closer_flag = 0
	get_farer_flag = 0
	out_of_band_flag = 0
	too_close_flag = 0
	open_turn_flag = 0

	print (check_if_obstacle_on_the_way_to_goal)


	current_angle_to_goal = check_goal_accessibility (position_x, position_y)
	if check_if_obstacle_on_the_way_to_goal(position_x, position_y, position_theta, directions) == False:
		#side_before_touch = "right"
		state = 1

	if state != 1:
		
		if exit_change_side_flag == 0:
			if already_touched_flag == 0:
				if len(list_of_touch_points) > 1:
					print "NUMBER OF TOUCH POINTS", number_of_touch_points
					print ("TOUCHED POINTS:")

					for point in list_of_touch_points:
						print point


					for iterator in range(0, len(list_of_touch_points)-1):

						distance_from_points = distance_from_2_points(list_of_touch_points[iterator][0], list_of_touch_points[iterator][1], position_x, position_y)
						if distance_from_points < 0.35 and number_of_touch_points > 1 and list_of_touch_points[iterator] not in list_of_visited_points:
							already_touched_flag = 1
							list_of_visited_points.append(list_of_touch_points[iterator])
							#already_touched_start_theta_position = position_theta
							end_theta = classify_touched_theta_position(position_theta)
							print ("Punkt juz odwiedzony")
							break
			if already_touched_flag == 1:
				start_side = "right"
				for i in range (0, 20):
					print ("TOUCHED")
				state = 6

		if already_touched_flag == 0 or exit_change_side_flag > 0:
			if check_head() == False :
				print "DIST TO WALL", dir
				print ("NUM IN LIST", num_in_list)

				if dir > 0.3 and dir < 0.5:
					open_turn_flag = 0
					if dir > 0.4 and dir < 0.5 and out_of_band_flag == 1:
						new_vel.angular.z = 0.2 * ((dir - 0.4)/0.2)
						new_vel.linear.x = 0.25

					elif dir > 0.3 and dir < 0.4 and too_close_flag == 1:
						new_vel.angular.z = -0.2 * ((0.4 - dir)/0.2)
						new_vel.linear.x = 0.25

					if (round(right_side_front, 2) == round(right_side_back, 2)):
						new_vel.linear.x = 0.4
						new_vel.angular.z = 0
						print ("LINE FOLLOW")
					else:
						if get_closer_flag == 0 and get_farer_flag == 0:
							if right_side_front >= right_side_back:
								new_vel.angular.z = -0.2
								new_vel.linear.x = 0.2
								print ("EQUALISE FROM FRONT TO BACK")
							else:
								new_vel.angular.z = 0.2
								new_vel.linear.x = 0.2
								print ("EQUALISE FROM BACK TO FRONT")
							return state


				elif dir > 0.5:
					open_turn_flag = 1
					new_vel.linear.x = 0
					if num_in_list > 280 and num_in_list < 300:
						new_vel.angular.z = 0
						new_vel.linear.x = 0.2
						print ("ROTATING TO WALL")
					else:
						new_vel.linear.x = 0
						new_vel.angular.z = -0.2
						print ("APPROACH DRIVE BAND")


				elif dir < 0.3:
					new_vel.angular.z = 0.2*((0.3-dir)/0.2)
					new_vel.linear.x = 0.3
			
			elif check_head() == True:
				new_vel.angular.z = 0
				new_vel.linear.x = 0
				state = 5
				theta_before_left_turn = position_theta
				theta_before_right_turn = position_theta
				print ("RIGHT TURN DETECTED")

		if exit_change_side_flag > 0:
			exit_change_side_flag -= 1



# funkcja nadpisuje wartość minimalnej odległości robota do celu
# funkcja jest wykorzystywana w algorytmach ALG2 i REV2
# jest wywoływana w momencie zmiany stanu z 1 na 3 ("dotknięcie sciany")
# jeżeli aktualna odległość do celu będzie mniejsza niż najmniejsza, 
# to wartość najmniejsza zostaje nadpisana wartością aktualną

def change_min_distance_to_goal(temp_distance):
	global min_distance_to_goal
	if temp_distance < min_distance_to_goal:
		min_distance_to_goal = temp_distance

# funkcja służy do obliczenia odległosći między dwoma punktami
# służy do sprawdzania warunku bliższej odległóści przy zmianie stany na 1 - jazda do celu
# służy także to sprawdzania odległości w algorytmie BUG2
# funkcja jest też wykorzystywana do sprawdzenia, czy robot dotarł do celu

def distance_from_2_points(touched_x, touched_y, current_x, current_y):
	two_points_distance = sqrt((touched_x-current_x)**2 + (touched_y-current_y)**2)
	return two_points_distance

# funkcja jest wywoływana w algorytmach ALG2 i REV2
# funkcja służy do zmiany kierunku jazdy robota, podczas gdy robot dotrze do punktu dojazdu do ściany (list_of_touch_points)

def change_side_if_touched(position_theta):
	global state
	global end_theta
	global exit_change_side_flag
	global current_side
	global previous_state_ALG2 
	global already_touched_flag
	print end_theta, "POZYCJA WYJSCIA"
	print position_theta, "AKTUALNA POYZJCA"

	if round(position_theta, 1) != round(end_theta, 1): # jeżeli aktualna pozycja nie równa się end theta -> pozycji wyjścia ze stanu 6 ustawionej w stanie 3 to robot obraca się
		new_vel.angular.z = 0.3
		new_vel.linear.x = 0
	else:
		new_vel.angular.z = 0
		new_vel.linear.x = 0

		if start_side == "right": # zmiana strony, którą robot okrąża ścianę z prawej na lewą
			current_side = "left"
		else:
			current_side = "right" # zmiana strony, którą robot okrąża ścianę z lewej na prawą
		# exit_change_side_flag = 0
		already_touched_flag = 0
		previous_state_ALG2 = 6
		state = 3 # zmiana stanu na 3 (jazda przy ścianie)

# funkcja wykorzystywana w algorytmach ALG2 i REV2
# służy do sprawdzenia, czy możliwy jest dojazd do celu, czy na drodze znajduje się przeszkoda


def check_if_obstacle_on_the_way_to_goal (position_x, position_y, position_theta, directions):
	global min_distance_to_goal
	global open_turn_flag
	global current_side

	goal_angle = check_goal_accessibility(position_x, position_y) # kąt do celu
	relative_angle_to_goal = (goal_angle - position_theta) # kąt do celu względem pozycji theta robota

	# wyznaczanie kąta do celu w miarze stopni 

	if relative_angle_to_goal < 0:
		degrees_angle = min(360 + int(relative_angle_to_goal/(2*pi)*360), 360)
	else:
		degrees_angle = min(int(relative_angle_to_goal/(2*pi)*360), 360)

	left_corner = correct_angle_value (degrees_angle + 45) # przypisanie zmienniej left_corner znormalizowanej wartości kąta degrees_angle + 45
	right_corner = correct_angle_value (degrees_angle + 315) # przypisanie zmienniej left_corner znormalizowanej wartości kąta degrees_angle + 135


	if min_distance_to_goal < distance_from_2_points(goal[0], goal[1], position_x, position_y): # jeżeli aktualna odległość do celu jest większa od minimalnej, to robot nie skieruje się w kierunku celu - nie zmieni stanu na 1
		return True

	else: # jeżeli aktualna odległość do celu jest mniejsza od minimalnej
		if open_turn_flag == 1:
			if directions[degrees_angle] > 0.7 and directions[left_corner] > 0.7 and directions[right_corner] > 0.7: # jeśli robot znajduje się w fazie skrętu otwartego(można to wywnioskować po wyjechaniu poza pas)
				return False # cel nie jest zaslonietny
			else:
				return True # cel zasloniety
				print "Przeszkoda na drodze skret otwarty"
		else: # jeżeli robot nie znajduje się w fazie skrętu otwartego - jedzie w pasie przy ścianie
			if current_side == "left":
					if degrees_angle > 200 and degrees_angle < 340 and directions [left_corner] > 0.7 and directions [right_corner] > 0.7:
						return False # cel nie jest zaslonietny
					else:
						return True # cel zasloniety
						print "Przeszkoda na drodze jazda lewym bokiem"

			elif current_side == "right":
				if degrees_angle > 20 and degrees_angle <160 and directions [left_corner] > 0.7 and directions [right_corner] > 0.7:
					return False # cel nie jest zaslonietny
				else:
					return True # cel zasloniety
					print "Przeszkoda na drodze jazda prawym bokiem"


# funkcja służy do normalizacji wartości kąta (0, 360)

def correct_angle_value(angle):
	if angle > 360:
		return angle-360
	else:
		return angle

# funkcja wykorzystywana w algorytmach ALG2 i REV2
# służy do wyznaczenia kat wyjściowego ze stanu 6 (obrót po dotarciu do punktu dojazdu do ściany) 

def classify_touched_theta_position(position_theta):
	if position_theta >= 0 and position_theta <= pi:
		end_theta = position_theta - pi
	else:
		end_theta = position_theta + pi
	return end_theta






# FUNKCJE UŻYWANE PRZY ALGORYTMIE REV2





# funkcja drive_to_finish_right_REV2 wykorzystywana w algorytmie REV2 
# funkcja jest wykonywana w stanie 1 i służy do jazdy do celu podczas,
# gdy robot okrąża labirynt prawą stroną do ściany



def drive_to_finish_right_REV2 (position_theta, position_x, position_y):
	global state
	global theta_before_right_turn
	global drive_to_finish_flag
	global angle_to_goal
	global number_of_touch_points
	global change_side_flag
	global current_side
	global list_of_touch_points
	global side_before_touch
	
    # global list_of_touch_points
	angle_to_goal = check_goal_accessibility(position_x, position_y)
	
	print left_side_min_quarter, "MIN_QUARTER"

	if abs(position_theta - angle_to_goal) > 0.1 and (right_side_min_quarter > 0.7 or drive_to_finish_flag == 0):
		if position_theta <= angle_to_goal:
			new_vel.angular.z = 0.3
		elif position_theta > angle_to_goal:
			new_vel.angular.z = -0.3
		new_vel.linear.x = 0
		state = 1
		print ("Rotating to goal angle")
	else:
		state = 1
		drive_to_finish_flag = 1
		print ("DISTOWALL", dir)
		print ("FRONT", right_side_front)
		print ("BACK", right_side_back)

		if  right_side_min_quarter > 0.5:
			new_vel.angular.z = 0
			new_vel.linear.x = 0.3
			print ("GO AHEAD")
		else:
			if len(list_of_touch_points) > 0: 
				if side_before_touch == "left": # różnaica wobec ALG2 i BUG 2 - zmiana strony okrążania ściany, przy dojeździe do ściany - robot musi wiedzie, w którą stronę się obrócić
					current_side = "right"
				elif side_before_touch == "right" or side_before_touch == "no_value":
					current_side = "left"

			if current_side == "right": # jeżeli robot będzie zaraz jechał prawą stroną przy ścianie
				if (abs(right_side_back - right_side_front) > 0.05 or right_side_perpendicular > 0.5):
					new_vel.angular.z = 0.3
					new_vel.linear.x = 0
					print ("ROTATE TO WALL FOLLOW")
				else:
					new_vel.linear.x = 0
					number_of_touch_points += 1
					list_of_touch_points.append((position_x, position_y))
					goal_distance = distance_from_2_points (goal[0], goal[1], position_x, position_y)
					change_min_distance_to_goal (goal_distance)
					previous_state = 1
					state = 3
			else:  # jeżeli robot będzie zaraz jechał lewą stroną przy ścianie
				if (abs(left_side_back - left_side_front) > 0.05 or left_side_perpendicular > 0.5):
					new_vel.angular.z = -0.3
					new_vel.linear.x = 0
					print ("ROTATE TO WALL FOLLOW")
				else:
					new_vel.linear.x = 0
					number_of_touch_points += 1
					list_of_touch_points.append((position_x, position_y))
					goal_distance = distance_from_2_points (goal[0], goal[1], position_x, position_y)	
					change_min_distance_to_goal (goal_distance)
					state = 3

# funkcja drive_to_finish_right_REV2 wykorzystywana w algorytmie REV2 
# funkcja jest wykonywana w stanie 1 i służy do jazdy do celu podczas,
# gdy robot okrąża labirynt lewą stroną do ściany


def drive_to_finish_REV2 (position_theta, position_x, position_y):
	global state
	global theta_before_right_turn
	global drive_to_finish_flag
	global angle_to_goal
	global number_of_touch_points
	global change_side_flag
	global current_side
	global list_of_touch_points
	global side_before_touch
	
    # global list_of_touch_points
	angle_to_goal = check_goal_accessibility(position_x, position_y)
	
	print left_side_min_quarter, "MIN_QUARTER"

	if abs(position_theta - angle_to_goal) > 0.1 and (left_side_min_quarter > 0.7 or drive_to_finish_flag == 0):
		if position_theta <= angle_to_goal:
			new_vel.angular.z = 0.3
		elif position_theta > angle_to_goal:
			new_vel.angular.z = -0.3
		new_vel.linear.x = 0
		state = 1
		print ("Rotating to goal angle")
	else:
		state = 1
		drive_to_finish_flag = 1
		print ("DISTOWALL", dir)
		print ("FRONT", left_side_front)
		print ("BACK", left_side_back)

		if  left_side_min_quarter > 0.5:
			new_vel.angular.z = 0
			new_vel.linear.x = 0.3
			print ("GO AHEAD")
		else:
			if len(list_of_touch_points) > 0:
				if side_before_touch == "left":
					current_side = "right"
				elif side_before_touch == "right" or side_before_touch == "no_value":
					current_side = "left"

			if current_side == "right":
				if (abs(right_side_back - right_side_front) > 0.05 or right_side_perpendicular > 0.5):
					new_vel.angular.z = 0.3
					new_vel.linear.x = 0
					print ("ROTATE TO WALL FOLLOW")
				else:
					new_vel.linear.x = 0
					number_of_touch_points += 1
					list_of_touch_points.append((position_x, position_y))
					goal_distance = distance_from_2_points (goal[0], goal[1], position_x, position_y)
					state = 3
			else:
				if (abs(left_side_back - left_side_front) > 0.05 or left_side_perpendicular > 0.5):
					new_vel.angular.z = -0.3
					new_vel.linear.x = 0
					print ("ROTATE TO WALL FOLLOW")
				else:
					new_vel.linear.x = 0
					number_of_touch_points += 1
					list_of_touch_points.append((position_x, position_y))
					goal_distance = distance_from_2_points (goal[0], goal[1], position_x, position_y)
					
					state = 3

# funkcja aktualizuje minimalną odległość z hitpointa do celu

def change_goal_distance():
	global list_of_touch_points
	global min_distance_to_goal
	temp_distance = 100
	for touch_point in list_of_touch_points:
		if distance_from_2_points(goal[0], goal[1], touch_point[0], touch_point[1]) < temp_distance:
			temp_distance = distance_from_2_points(goal[0], goal[1], touch_point[0], touch_point[1])
	min_distance_to_goal = temp_distance

# funkcja zmieniająca aktualną stronę jazdy przy dotarciu do następnej ściany

def change_side():
	global current_side
	if current_side == "left":
		current_side = "right"
	elif current_side == "right" or current_side == "no_value":
		current_side = "left"
	return current_side
		


# funkcja wywolywana przy przyjsciu danych o lokalizacji robota

def odom_callback(odom):
	global new_vel
	global distance_to_goal
	global min_distance_to_goal
	global list_of_touch_points
	global list_of_visited_points
	global algorithm_choice
	global side_before_touch
	global state
	pose = Pose()
	pose.x = odom.pose.pose.position.x
	pose.y = odom.pose.pose.position.y
	pose.theta = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])[2]
	
	print ("CURRENT SIDE", current_side)

	print ("LIST OF TOUCH POINTS")

	for point in list_of_touch_points:
		print point

	print ("LIST OF VISITED POINTS")

	for point in list_of_visited_points:
		print point

	print ("SIDE BEFORE TOUCH", side_before_touch)

	print "Min odleglosc do celu", min_distance_to_goal
	print "Pozycja x: ",odom.pose.pose.position.x
	print "Pozycja y: ",odom.pose.pose.position.y
	print "Pozycja theta: ",pose.theta
	print "Kat do celu:", angle_to_goal
	print " Odleglosc od sciany", head
	print "STAN", state

	if check_if_in_finish(pose.x, pose.y): # jeżeli robot znajdzie się u celu
		print check_if_in_finish(pose.x, pose.y) # program zwraca informację o osiągnięciu celu

	# wybór algorytmu na podstawie decyzji użytkownika
	else:
		if algorithm_choice == "BUG":
			BUG2(pose.x, pose.y, pose.theta)
		elif algorithm_choice == "ALG":
			ALG2(pose.x, pose.y, pose.theta, list_of_ranges)
		elif algorithm_choice == "REV":
			REV2(pose.x, pose.y, pose.theta, list_of_ranges)

if __name__== "__main__":
	global new_vel
	global algorithm_choice
	global current_side
	global alg
	global goal
	alg = input("Prosze wybrac algorytm. BUG/ALG/DEV")
	x_coeff = raw_input("Wspolrzedna X celu:")
	y_coeff = raw_input("Wspolrzedna Y celu:")
	algorithm_choice = raw_input("Prosze wybrac algorytm. BUG/ALG/DEV")
	current_side = raw_input("Prosze wybrac poczatkowy kierunek jazdy (ktora storna przy scianie. right/left")
	goal = (int(x_coeff), int(y_coeff))
	new_vel = Twist()
	rospy.init_node('wr_zad', anonymous=True)
	print("ready")
	rospy.Subscriber( '/odom' , Odometry, odom_callback)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber( '/scan' , LaserScan, scan_callback)
	
	rate=rospy.Rate(10) # 10Hz
	while not rospy.is_shutdown():
		pub.publish(new_vel)#wyslanie predkosci zadanej
		rate.sleep()

	print("END")
