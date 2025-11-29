from django.shortcuts import render, redirect
from .forms import PhotoUploadForm
from django.conf import settings
from django.http import HttpResponse, HttpResponseBadRequest
import os
import cv2
from deepface import DeepFace
import numpy as np


def select_images(request):
    print("VIEW LOADED FROM:", os.path.dirname(__file__))
    # --------- 1. CHECK IF BACKEND HAS SET THE FOLDER NAME ---------
    folder_name = request.session.get("generated_folder")
    print(f"{folder_name}")
    if not folder_name:
        # No folder yet → backend hasn’t provided it
        return render(request, "data_sel/waiting_for_folder.html")

    # Build absolute path
    folder_path = os.path.join(settings.MEDIA_ROOT,folder_name)
    print(f"{settings.MEDIA_ROOT}")
    print(f"our path{folder_name}")
    print(f" folder path {folder_path}")
    # Safety check
    if not os.path.exists(folder_path):
        return HttpResponseBadRequest(f"Folder '{folder_name}' does not exist.")

    # --------- 2. GET ALL IMAGES IN THE FOLDER ---------
    image_files = [
        f for f in os.listdir(folder_path)
        if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif'))
    ]
    image_files.sort()

    # --------- 3. USER SUBMITTED SELECTED IMAGES ---------
    if request.method == "POST":
        selected_images = request.POST.getlist("selected_images")

        # Send selected image names to backend function
        # (You can replace this with anything you need)
        return render(
            request,
            "data_sel/selected_images.html",
            {
                "selected_images": selected_images,
                "folder": folder_name
            }
        )

    # --------- 4. SHOW IMAGES FOR SELECTION ---------
    return render(
        request,
        "data_sel/select_images.html",
        {
            "folder": folder_name,
            "images": image_files
        }
    )

def upload_photo(request):
    request.session.pop("generated_folder", None)
    if request.method == 'POST':
        form = PhotoUploadForm(request.POST, request.FILES)
        if form.is_valid():
            upload_image=form.save()
            extract_faces(upload_image,request)
            return redirect('select_images')  # Redirect to a success page after upload
    else:
        form = PhotoUploadForm()
    return render(request, "data_sel/upload_photo.html", {'form': form})

def success(request):
    return HttpResponse("Photo uploaded successfully!")



def extract_faces(upload_image,request):
    output_dir="faces"
    image_path= upload_image.image.path
    image_path_temp= image_path.split('.')[0]
    total_path=os.path.join(image_path_temp,output_dir)
    print(f"{image_path}")
    detector_backend = "opencv" 
    os.makedirs(total_path, exist_ok=True)
    
    detected_faces = DeepFace.extract_faces(img_path=image_path, detector_backend='opencv')

# Create a directory to save the faces

# Save each detected face
    for i, face in enumerate(detected_faces):
        face_img = face['face']
        face_img = (face_img * 255).astype(np.uint8)
        # Convert the face image from RGB to BGR (OpenCV format)
        face_img = cv2.cvtColor(face_img, cv2.COLOR_RGB2BGR)
        # Save the face image
        face_filename = os.path.join(total_path, f"face_{i+1}.jpg")
        print(f"{face_filename}")
        cv2.imwrite(face_filename, face_img)
        print(f"Saved face_{i+1}.jpg")
    print(f"Total faces extracted: {len(detected_faces)}")
    media_folder= total_path.split("media/")[-1]
    selecting_folder(request,folder_past=media_folder)


def selecting_folder(request,folder_past):
    request.session["generated_folder"] = folder_past
