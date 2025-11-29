import os
from django.db import models
from datetime import datetime

def timestamped_filename(instance, filename):
    # Get file extension
    ext = filename.split('.')[-1]
    name= filename.split('.')[0]
    # Create timestamp string
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Build new filename
    new_filename = f"{name}_{timestamp}.{ext}"

    # Save inside "photos/" directory
    return os.path.join("photos", new_filename)



# Create your models here.
class Photo(models.Model):
    title = models.CharField(max_length=255, blank=True)
    image = models.ImageField(upload_to=timestamped_filename)
    uploaded_at = models.DateTimeField(auto_now_add=True)

    def __str__(self):
        return self.title or f"Photo {self.id}"