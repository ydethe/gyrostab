import requests




if __name__ == '__main__':
    url = "eru.synology.me:5000"
    response = requests.get(url+'/imu')
    assert response.status_code == 200 # A quoi ca sert ??
    print(response.json())
    
    response = requests.put(url+'/servo', json = {'x': 20, 'y': 140, 'z': 75})
    assert response.status_code == 200
    print(response.json())
    
