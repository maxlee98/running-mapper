from flask import Flask, jsonify, request
import googlemaps
from geopy.distance import geodesic
import math

app = Flask(__name__)

# Initialize Google Maps client
gmaps = googlemaps.Client(key='AIzaSyBmM4WPHKrWrtApji5bANpeishBx7Re68k')

# Haversine formula to calculate distance between two lat/lon points
def haversine(lat1, lon1, lat2, lon2):
    radius = 6371000  # Earth radius in meters
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return radius * c

# Function to generate waypoints along the circle
def generate_waypoints(center_lat, center_lng, radius, num_points):
    waypoints = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        lat_offset = (radius / 6371000) * math.sin(angle)  # Latitude offset in radians
        lon_offset = (radius / 6371000) * math.cos(angle) / math.cos(math.radians(center_lat))  # Longitude offset in radians
        new_lat = center_lat + math.degrees(lat_offset)
        new_lng = center_lng + math.degrees(lon_offset)
        waypoints.append((new_lat, new_lng))
    return waypoints

# Function to find the closest walkable paths to a given waypoint
def get_closest_walkable_path(lat, lng):
    places_result = gmaps.places_nearby((lat, lng), radius=50, type='route')
    if places_result['results']:
        return places_result['results'][0]['geometry']['location']
    return None

# Function to get a walking route between two points
def get_walking_route(start, end):
    directions_result = gmaps.directions(start, end, mode="walking")
    if directions_result:
        return directions_result[0]['legs'][0]['steps']
    return []

@app.route('/generate_route', methods=['POST'])
def generate_route():
    data = request.json
    start_lat = data.get('start_lat')
    start_lng = data.get('start_lng')
    radius = data.get('radius')  # In meters
    num_waypoints = data.get('num_waypoints', 8)  # Number of points along the circle
    
    # Generate waypoints along the circle
    waypoints = generate_waypoints(start_lat, start_lng, radius, num_waypoints)
    
    # Get walking routes for each pair of consecutive waypoints
    routes = []
    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        end = waypoints[i + 1]
        
        # Get the closest walkable paths near the waypoints
        start_nearby = get_closest_walkable_path(start[0], start[1])
        end_nearby = get_closest_walkable_path(end[0], end[1])
        
        if start_nearby and end_nearby:
            # Get the walking route between the closest walkable paths
            route = get_walking_route(start_nearby, end_nearby)
            routes.append(route)
    
    return jsonify({
        'waypoints': waypoints,
        'routes': routes
    })

if __name__ == '__main__':
    app.run(debug=True)
