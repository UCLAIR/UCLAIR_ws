import folium
def zigzag_area(num_zigzags):
    # Define the coordinates of the area to scout
    area_coords = [(38.31442311312976, -76.54522971451763),
                   (38.31421041772561, -76.54400246436776),
                   (38.3144070396263, -76.54394394383165),
                   (38.31461622313521, -76.54516993186949),
                   (38.31442311312976, -76.54522971451763)]
    
    # Calculate the total number of segments in the zigzag path
    num_segments = (len(area_coords) - 1) * num_zigzags
    
    # Calculate the length of each segment
    segment_length = 1.0 / num_segments
    
    # Initialize the list of GPS coordinates for the drone's path
    path_coords = []
    
    # Iterate over the segments of the zigzag path
    for i in range(num_segments):
        # Calculate the index of the segment in the original area coordinates
        segment_index = i // num_zigzags
        
        # Calculate the fraction of the way along the segment the drone should be
        segment_fraction = (i % num_zigzags) / num_zigzags
        
        # Calculate the GPS coordinates of the point on the segment at the specified fraction
        lat1, lon1 = area_coords[segment_index]
        lat2, lon2 = area_coords[segment_index + 1]
        lat = lat1 + segment_fraction * (lat2 - lat1)
        lon = lon1 + segment_fraction * (lon2 - lon1)
        
        # Add the GPS coordinates to the path list
        path_coords.append((lat, lon))
    
    # Return the list of GPS coordinates for the drone's path
    return path_coords


if __name__ == "__main__":
    # Define the center of the map and create the map object
    map_center = [38.31442311312976, -76.54522971451763]
    m = folium.Map(location=map_center, zoom_start=18)

    # Get the list of GPS coordinates for the drone's path
    path_coords = zigzag_area(3)

    # Create a list of lat-lon pairs for the polyline
    polyline_coords = [(lat, lon) for lat, lon in path_coords]

    # Add the polyline to the map
    folium.PolyLine(locations=polyline_coords, color='red').add_to(m)

    # Display the map
    m




