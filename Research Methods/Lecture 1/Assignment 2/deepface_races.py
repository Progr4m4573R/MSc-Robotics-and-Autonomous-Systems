from deepface import DeepFace
from collections import Counter
import os

# folder path
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
img_dir = os.path.join(BASE_DIR, "lfw-deepfunneled/lfw-deepfunneled/")

#Count the number of people in the dataset via the folder containing their pictures 
#source: https://pynative.com/python-count-number-of-files-in-a-directory/#h-how-to-count-files-in-a-directory
folder_count = 0
file_count = 0
# Iterate directory
for path in os.listdir(img_dir):
    # check if current path is a file
    if os.path.isfile(os.path.join(img_dir, path)):
        file_count += 1
    else:
        folder_count+=1
print('Folder count: ', folder_count,"\nFile count: ",file_count)

dataset_size = folder_count
races_count = []

#Iterate through images folder and get race and name data
for root,dirs, files in os.walk(img_dir):
    for file in files:
        if file.endswith("png") or file.endswith("jpg"):
            path = os.path.join(root,file)
            try:
                #Only using one picture prevents one person's ethnicity from being counted more than once.
                if '0001' in path:
                    obj = DeepFace.analyze(img_path = path,
                    actions = ['age', 'gender', 'race', 'emotion'])
                    #Picks the most likely race the person belongs to by highest percentage race value
                    race = obj["dominant_race"]
                    print("Race: ",obj["dominant_race"], "Name: ",file)
                    races_count.append(race)
            
            except Exception as e:
                print("Error detected",e,"/nLocation: ", path)
                races_count.append("race_unknown")

#Print file location for debugging
#print("File location ",file)
#Print array of races
print(races_count)
#Counting occurences:
#https://sparkbyexamples.com/python/count-occurrences-of-element-in-python-list/#:~:text=To%20count%20the%20occurrences%20of%20an%20element%20in%20a%20list,of%20elements%20in%20a%20list.
white = Counter(races_count)["white"]
asian = Counter(races_count)["asian"]
latino_hispanic = Counter(races_count)["latino hispanic"]
middle_eastern = Counter(races_count)["middle eastern"]
black = Counter(races_count)["black"]
indian = Counter(races_count)["indian"]
unknown = Counter(races_count)["race_unkown"]

print("Percentage of White ethnicity in dataset",white/(dataset_size)*100, "%")
print("Percentage of Black ethnicity in dataset",black/(dataset_size)*100, "%")
print("Percentage of Asian ethnicity in dataset",asian/(dataset_size)*100, "%")
print("Percentage of Middle Eastern ethnicity in dataset",middle_eastern/(dataset_size)*100, "%")
print("Percentage of Indian ethnicity in dataset",indian/(dataset_size)*100, "%")
print("Percentage of Latino Hispanic ethnicity in dataset",latino_hispanic/(dataset_size)*100, "%")
print("Percentage of races unrecognised",unknown/(dataset_size)*100,"%")