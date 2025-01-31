import googlemaps
from geopy.distance import geodesic
import folium

# Initialize the Google Maps client
gmaps = googlemaps.Client(key="AIzaSyBmM4WPHKrWrtApji5bANpeishBx7Re68k")

def get_route(start_point, end_point, mode="walking"):
    """Get a route from start_point to end_point using the Directions API."""
    directions = gmaps.directions(start_point, end_point, mode=mode)
    if not directions:
        return None
    return directions[0]

def calculate_route_distance(route):
    """Calculate the total distance of a route in meters."""
    legs = route["legs"]
    total_distance = sum(leg["distance"]["value"] for leg in legs)
    return total_distance

# def adjust_endpoint(start_point, initial_endpoint, target_distance, tolerance=100):
#     """Adjust the endpoint to achieve a route close to the target distance."""
#     current_distance = 0
#     adjusted_endpoint = initial_endpoint

#     while abs(current_distance - target_distance) > tolerance:
#         route = get_route(start_point, adjusted_endpoint)
#         if not route:
#             break

#         current_distance = calculate_route_distance(route)
#         print(f"Current distance: {current_distance} meters")

#         # Adjust the endpoint based on the difference
#         distance_diff = target_distance - current_distance
#         direction_vector = (
#             adjusted_endpoint[0] - start_point[0],
#             adjusted_endpoint[1] - start_point[1],
#         )
#         scale_factor =  (distance_diff / current_distance * 0.9)

#         adjusted_endpoint = (
#             start_point[0] + direction_vector[0] * scale_factor,
#             start_point[1] + direction_vector[1] * scale_factor,
#         )

#     return route

def adjust_endpoint(start_point, initial_endpoint, target_distance, tolerance=100, max_iterations=100):
    """Adjust the endpoint to achieve a route close to the target distance."""
    current_distance = 0
    adjusted_endpoint = initial_endpoint
    iterations = 0

    while abs(current_distance - target_distance) > tolerance and iterations < max_iterations:
        if iterations % 200 == 0:
            print(print(f"Current Iteration: {iterations} | Distance: {current_distance} meters"))

        route = get_route(start_point, adjusted_endpoint)
        if not route:
            break

        current_distance = calculate_route_distance(route)
        # print(f"Current distance: {current_distance} meters")

        # Adjust the endpoint in smaller increments
        distance_diff = target_distance - current_distance
        step_size = distance_diff * 0.01  # Make small adjustments to avoid overshooting

        # Move the endpoint gradually towards the target distance
        direction_vector = (
            adjusted_endpoint[0] - start_point[0],
            adjusted_endpoint[1] - start_point[1],
        )
        scale_factor = 1 + step_size / current_distance
        adjusted_endpoint = (
            start_point[0] + direction_vector[0] * scale_factor,
            start_point[1] + direction_vector[1] * scale_factor,
        )
        
        iterations += 1

    # print(route)
    return route


def plot_route(route, route_name):
    """Plot the route on a map using folium."""
    start_point = route["legs"][0]["start_location"]
    m = folium.Map(location=[start_point["lat"], start_point["lng"]], zoom_start=14)

    # Add the route to the map
    steps = route["legs"][0]["steps"]
    for step in steps:
        start = step["start_location"]
        end = step["end_location"]
        html_instructions = step['html_instructions']
        print(start, end, html_instructions)
        folium.PolyLine(
            [(start["lat"], start["lng"]), (end["lat"], end["lng"])],
            color="blue",
            weight=2.5,
            opacity=1,
        ).add_to(m)

    m.save("{}.html".format(route_name))


if __name__ == "__main__":

    # Example usage
    # start_point = (37.7749, -122.4194)  # San Francisco
    # initial_endpoint = (37.7849, -122.4144)  # Nearby location
    start_point = (1.402162, 103.746971)  # San Francisco
    initial_endpoint = (1.402098, 103.747225)  # Nearby location
    target_distance = 5000  # 5km in meters
    route_name = "CCK Route {} m".format(target_distance)

    route = adjust_endpoint(start_point, initial_endpoint, target_distance)
    if route:
        print(f"Final route distance: {calculate_route_distance(route)} meters")
        plot_route(route, route_name)
        print("Route saved to {}.html".format(route_name))
    else:
        print("Failed to generate a route.")