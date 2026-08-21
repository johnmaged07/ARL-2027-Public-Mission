#ASU Racing team solo mission#
def monitor_following_distance(distances: list[float], speeds: list[float]) -> tuple[int, float, int]:
    Tailgating=False
    tailgating_seconds=0
    tailgate_incidents=0
    for i in range(len(distances)):
        if distances[i] < speeds[i]*0.5:
            tailgating_seconds+=1
            if Tailgating==False:
                Tailgating=True
                tailgate_incidents+=1
        else:
            Tailgating=False

    if distances != []:
        minimum_distance=min(distances)
    else:
        minimum_distance=0.0
        
    return (tailgating_seconds,minimum_distance,tailgate_incidents) 




