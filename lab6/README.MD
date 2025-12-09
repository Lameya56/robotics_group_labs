<h1>Lab Report</h1>
https://docs.google.com/document/d/17FS_6bGWh9pmp1LpHMu_oumwGzT8tCWpS5gHNZDOLcg/edit?tab=t.0

<h2>Dependencies</h2>
Install required Python packages:
pip install -r requirements.txt

<h2>How to Run the Code</h2>
1. Using a Video File located in Data folder:
- Modify main.py on lines 17-19:

> video_path = "data/your_video.mp4"
>
>test_name = "your_test_name"
>
>save_annotated_video = True

2. Using a Live Webcam: 
- Modify main.py on line 31:

>cap = cv2.VideoCapture(0)

- Comment out main.py line 17

<h2>How to Switch TARGET_CLASS_ID for MobileNet-SSD</h2>
- Modify TARGET_CLASS_ID in main.py on line 10:

>TARGET_CLASS_ID = 5  # # class ID for 'bottle'

Valid TARGET_CLASS_ID ranges from 0-20:
| Class ID | Object Class | Class ID | Object Class |
|----------|--------------|----------|--------------|
| 0 | background | 11 | diningtable |
| 1 | aeroplane | 12 | dog |
| 2 | bicycle | 13 | horse |
| 3 | bird | 14 | motorbike |
| 4 | boat | 15 | person |
| 5 | bottle (default) | 16 | pottedplant |
| 6 | bus | 17 | sheep |
| 7 | car | 18 | sofa |
| 8 | cat | 19 | train |
| 9 | chair | 20 | tvmonitor |
| 10 | cow | | |



