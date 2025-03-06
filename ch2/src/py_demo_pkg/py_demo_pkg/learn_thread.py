import threading
import requests

class Download:
    def __init__(self):
        pass
    
    def download(self, url, callback):
        print(f"Thread {threading.get_ident()} start downloading: {url}")
        response = requests.get(url)
        response.encoding = 'utf-8'
        callback(url, response.text)
        
    def start_download(self, url, callback):
        thread = threading.Thread(target=self.download, args=(url, callback))
        thread.start()
        

def download_finished_callback(url, result):
    print(f"{url} download finished, total nums: {len(result)}, content: {result[:5]}...")
    

def main():
    d = Download()
    d.start_download('http://localhost:8000/novel1.txt', download_finished_callback)
    d.start_download('http://localhost:8000/novel2.txt', download_finished_callback)
    d.start_download('http://localhost:8000/novel3.txt', download_finished_callback)
    
    
        
    