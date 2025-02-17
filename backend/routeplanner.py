# TO DO:
# Use the optimised directions from google map
# Use the polyline given within the google map directions response. (DONE)
# Use the snap to closest road to prevent the route from going into unauthorised areas


from dotenv import load_dotenv
import os

import math
import googlemaps
import folium
from folium.plugins import AntPath
# For code documentation
from typing import Dict

# Load the environment variables from .env file
load_dotenv()

# Now you can access the environment variable just like before
google_map_api = os.environ.get('GOOGLE_MAPS_API')
start_lat, start_lng = os.environ.get("start_lat"), os.environ.get("start_lng")
# Initialize Google Maps client
gmaps = googlemaps.Client(key=google_map_api)

# Earth's radius in meters
EARTH_RADIUS = 6371000

class RunningRoutePlanner():

    def calculate_dist(pt1, pt2):
        # Haversine formula to calculate distance between two lat/lng points
        R = 6371 * 1000  # Radius of Earth in km
        lat1, lon1 = math.radians(pt1["lat"]), math.radians(pt1["lng"])
        lat2, lon2 = math.radians(pt2["lat"]), math.radians(pt2["lng"])

        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Distance in kilometers
        distance = R * c
        return distance

    # Function to get walking route using Google Maps API
    def get_walking_route(start, end):
        directions_result = gmaps.directions(start, end, mode="walking")
        if directions_result:
            return directions_result[0]['legs'][0]['steps']
        return []
    
    def get_full_route(waypoints):
        routes = []
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            
            # Get walking route
            route = RunningRoutePlanner.get_walking_route(start, end)
            routes.append(route)
        return routes

    def plot_route(waypoints, colour = "blue", route_name="Default"):
        """Plot the route on a map using folium."""
    
        start_point = waypoints[0]
        # Ensure that the plot is closed in a full circle if going back to the start

        # Creates the starting point of the map
        m = folium.Map(location=[start_point['lat'], start_point['lng']], zoom_start=14)

        # Route from all waypoints
        for i in range(len(waypoints)-1):
            start = waypoints[i]
            end = waypoints[i+1]
            # Print start and end coordinates from waypoints

            # Optional: Add a Circle Marker for more visibility
            folium.CircleMarker(
                location=[start['lat'], start['lng']],
                radius=5 if i % 10 == 0 else 1,
                color=colour,
                fill=True if i % 10 == 0 else False,
                fill_color="cyan",
                fill_opacity=0.7,
                tooltip= f"{i}, Lat: {start['lat']}, Lng: {start['lng']},"
            ).add_to(m)

            folium.PolyLine(
                [(start['lat'], start['lng']), (end['lat'], end['lng'])],
                color=colour,
                weight=2.5,
                opacity=1,
            ).add_to(m)

        m.save(os.path.join(os.getcwd(), "running-mapper", "generated_routes", "{}.html".format(route_name)))


# For calculating distance to see if overshot the intended distance.
class CircularRunningRoutePlanner(RunningRoutePlanner):

    def __init__(self, starting_pt: Dict[str, float] , running_distance:float, num_major_waypoints:int, closed_loop:bool):
        self.lat, self.lng = starting_pt['lat'], starting_pt['lng']
        self.running_distance = running_distance
        self.radius = self.calculate_radius(self.running_distance)
        self.center_lat, self.center_lng = self.calculate_center(start_point="top")
        self.major_waypoints = self.generate_major_waypoints(num_points=num_major_waypoints)
        self.plot_route(self.major_waypoints, colour="blue", route_name="TestRunningRoute", closed_loop=True)
        

    @staticmethod
    def calculate_radius(distance=5000):
        radius = distance / (2 * math.pi)
        # print(f"Radius of the circle: {radius:.2f} meters")
        return radius


    # Function to calculate the center of the circle given a starting point and radius
    def calculate_center(self, start_point="top"):
        radius = CircularRunningRoutePlanner.calculate_radius(self.running_distance)
        # Calculate the change in latitude (in radians)
        delta_lat = radius / EARTH_RADIUS
        delta_lat_deg = math.degrees(delta_lat)

        # Calculate the change in longitude (in radians)
        delta_lon = radius / (EARTH_RADIUS * math.cos(math.radians(lat_s)))
        delta_lon_deg = math.degrees(delta_lon)

        # If the starting point is at the top of the circle (North), move down by radius
        center_lat = self.lat - delta_lat_deg if start_point == "top" else self.lat + delta_lat_deg
        center_lng = self.lng  # Longitude doesn't change as the circle is centered

        return center_lat, center_lng
    

    def generate_major_waypoints(self, num_points):
        waypoints = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points + (math.pi / 2) # +pi/2 here as radians start on the right quad, but we want to start at 360 degrees
            lat_offset = (self.radius / EARTH_RADIUS) * math.sin(angle)  # Latitude offset in radians
            lon_offset = (self.radius / EARTH_RADIUS) * math.cos(angle) / math.cos(math.radians(self.center_lat))  # Longitude offset in radians
            new_lat = self.center_lat + math.degrees(lat_offset)
            new_lng = self.center_lng + math.degrees(lon_offset)
            waypoints.append({"lat":new_lat, "lng":new_lng})
        return waypoints

    def plot_route(self, waypoints, colour="red", route_name="CircularRoute", closed_loop=True):
        """Calls the parent method to plot the circular route"""
        if closed_loop:
            waypoints.append(waypoints[0])
        super().plot_route(waypoints, colour, route_name)



if __name__ == "__main__":

    # Example usage
    # start_point = (37.7749, -122.4194)  # San Francisco
    # initial_endpoint = (37.7849, -122.4144)  # Nearby location
    start_point = (1.402162, 103.746971)  # San Francisco
    initial_endpoint = (1.402098, 103.747225)  # Nearby location
