import osmnx as ox
import networkx as nx
from flask import Flask, request, jsonify
from flask_cors import CORS
import math
from geopy.distance import geodesic

app = Flask(__name__)
CORS(app)

def calculate_bbox(lat1, lon1, lat2, lon2, buffer=0.03):
    north = max(lat1, lat2) + buffer
    south = min(lat1, lat2) - buffer
    east = max(lon1, lon2) + buffer
    west = min(lon1, lon2) - buffer
    return north, south, east, west

def convert_to_kmh(max_speed):
    if isinstance(max_speed, str):
        max_speed = max_speed.lower()
        if "mph" in max_speed:
            return float(max_speed.replace("mph", "").strip()) * 1.60934
        else:
            return float(max_speed.replace("km/h", "").strip())
    elif isinstance(max_speed, list):
        return convert_to_kmh(max_speed[0])
    return float(max_speed)

def add_edge_costs(G):
    for u, v, data in G.edges(data=True):
        length_km = data.get('length', 1) / 1000  # Convertir longitud a kilómetros
        max_speed = data.get('maxspeed', 50)  # Velocidad máxima por defecto de 50 km/h si no está disponible
        max_speed_kmh = convert_to_kmh(max_speed)
        data['time'] = length_km / max_speed_kmh * 60  # Tiempo en minutos
        data['fuel_cost'] = length_km * 0.1  # Costo de combustible (arbitrario)
        data['toll_cost'] = data.get('toll_cost', 0)  # Ajustar según los datos reales de peajes, por defecto 0

def heuristic(a, b, G):
    coords_1 = (G.nodes[a]['y'], G.nodes[a]['x'])
    coords_2 = (G.nodes[b]['y'], G.nodes[b]['x'])
    distance_km = geodesic(coords_1, coords_2).km
    average_speed_kmh = 50.0
    time_estimate = (distance_km / average_speed_kmh) * 60
    return time_estimate

def cost_function(u, v, data, avoid_tolls, optimize_fuel):
    toll_cost = data.get('toll_cost', 0) if avoid_tolls else 0
    fuel_cost = data.get('fuel_cost', 0) if optimize_fuel else 0
    time_cost = data.get('time', 0)
    return time_cost + toll_cost + fuel_cost

def a_star_search(G, start, goal, avoid_tolls=False, optimize_fuel=False):
    path = nx.astar_path(G, start, goal, heuristic=lambda a, b: heuristic(a, b, G),
                         weight=lambda u, v, d: cost_function(u, v, d, avoid_tolls, optimize_fuel))
    return path

def calculate_total_distance_and_time(G, path):
    total_distance = 0  # en metros
    total_time = 0  # en minutos
    for i in range(len(path) - 1):
        u, v = path[i], path[i + 1]
        edge_data = G.get_edge_data(u, v)
        # Suponiendo que siempre hay un solo diccionario de datos por arista
        data = edge_data[0]
        total_distance += data['length']
        total_time += data['time']
    return total_distance / 1000, total_time  # convertir distancia a kilómetros y tiempo ya está en minutos

def get_secondary_routes(G, start, goal, main_distance):
    try:
        # Convertir a un grafo simple para usar shortest_simple_paths
        G_simple = nx.Graph()
        for u, v, data in G.edges(data=True):
            if not G_simple.has_edge(u, v):
                G_simple.add_edge(u, v, **data)
            else:
                # Mantener la arista con el menor peso
                if G_simple[u][v]['length'] > data['length']:
                    G_simple[u][v].update(data)

        all_paths = nx.shortest_simple_paths(G_simple, start, goal, weight='length')
        secondary_routes = []
        for path in all_paths:
            if len(secondary_routes) >= 2:
                break
            distance, time = calculate_total_distance_and_time(G, path)
            if distance <= 2 * main_distance:
                secondary_routes.append((path, distance, time))
        return secondary_routes
    except nx.NetworkXNoPath:
        return []

@app.route('/route', methods=['GET'])
def get_route():
    try:
        start_lat = float(request.args.get('start_lat'))
        start_lon = float(request.args.get('start_lon'))
        goal_lat = float(request.args.get('goal_lat'))
        goal_lon = float(request.args.get('goal_lon'))
        avoid_tolls = request.args.get('avoid_tolls', 'false').lower() == 'true'
        optimize_fuel = request.args.get('optimize_fuel', 'false').lower() == 'true'

        buffer = 0.03  # Tamaño inicial del buffer

        while buffer <= 1.0:  # Limitar el tamaño máximo del buffer
            try:
                # Calcular el bounding box para cubrir el origen y el destino
                north, south, east, west = calculate_bbox(start_lat, start_lon, goal_lat, goal_lon, buffer)
                bbox = (north, south, east, west)

                # Descargar el grafo de calles para el bounding box
                G = ox.graph_from_bbox(*bbox, network_type='drive')

                # Añadir costes a las aristas
                add_edge_costs(G)

                # Encontrar los nodos más cercanos a los puntos de inicio y destino
                start_node = ox.nearest_nodes(G, start_lon, start_lat)
                goal_node = ox.nearest_nodes(G, goal_lon, goal_lat)

                # Calcular la ruta principal
                main_route = a_star_search(G, start_node, goal_node, avoid_tolls, optimize_fuel)
                main_distance, main_time = calculate_total_distance_and_time(G, main_route)
                main_average_speed = (main_distance / (main_time / 60)) if main_time > 0 else 0  # velocidad en km/h

                # Supongamos que obtienes el precio de la gasolina de una API o de otro servicio
                gas_price_per_liter = 21.9  # Precio de la gasolina en pesos mexicanos por litro (valor de ejemplo)

                # Calcular el costo estimado de gasolina para la ruta principal
                main_fuel_cost = sum(G[u][v][0]['fuel_cost'] for u, v in zip(main_route[:-1], main_route[1:]))
                main_fuel_cost_in_pesos = main_fuel_cost * gas_price_per_liter

                # Calcular rutas secundarias
                secondary_routes = get_secondary_routes(G, start_node, goal_node, main_distance)
                secondary_routes_info = []
                for route, distance, time in secondary_routes:
                    average_speed = (distance / (time / 60)) if time > 0 else 0  # velocidad en km/h
                    route_coords = [(G.nodes[n]['y'], G.nodes[n]['x']) for n in route]
                    fuel_cost = sum(G[u][v][0]['fuel_cost'] for u, v in zip(route[:-1], route[1:]))
                    fuel_cost_in_pesos = fuel_cost * gas_price_per_liter
                    secondary_routes_info.append({
                        "route": route_coords,
                        "total_distance_km": distance,
                        "total_time_min": time,
                        "average_speed_kmh": average_speed,
                        "fuel_cost_pesos": fuel_cost_in_pesos
                    })

                main_route_coords = [(G.nodes[n]['y'], G.nodes[n]['x']) for n in main_route]
                return jsonify({
                    "main_route": {
                        "route": main_route_coords,
                        "total_distance_km": main_distance,
                        "total_time_min": main_time,
                        "average_speed_kmh": main_average_speed,
                        "fuel_cost_pesos": main_fuel_cost_in_pesos
                    },
                    "secondary_routes": secondary_routes_info
                })
            except nx.NetworkXNoPath:
                buffer += 0.05  # Incrementar el tamaño del buffer si no se encuentra una ruta
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        return jsonify({"error": "No route found within reasonable distance"}), 404
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/validate-point', methods=['GET'])
def validate_point():
    try:
        lat = float(request.args.get('lat'))
        lon = float(request.args.get('lon'))
        
        # Calcular el bounding box para cubrir el área alrededor del punto
        buffer = 0.005
        north, south, east, west = calculate_bbox(lat, lon, lat, lon, buffer)
        bbox = (north, south, east, west)

        # Descargar el grafo de calles para el bounding box
        G = ox.graph_from_bbox(*bbox, network_type='drive')

        # Encontrar el nodo más cercano al punto
        nearest_node = ox.nearest_nodes(G, lon, lat)
        nearest_node_coords = (G.nodes[nearest_node]['y'], G.nodes[nearest_node]['x'])
        point_coords = (lat, lon)
        distance = geodesic(point_coords, nearest_node_coords).meters

        # Definir un umbral de distancia para validar el punto
        threshold_distance = 50  # metros
        is_valid = distance <= threshold_distance

        return jsonify({"valid": is_valid})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True, host="0.0.0.0", port=5000)

