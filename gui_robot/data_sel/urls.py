
from django.urls import path
from  data_sel import views 
urlpatterns = [
    path('upload/', views.upload_photo, name='upload_photo'),
    path('success/', views.success, name='success'),
    path("select_images/", views.select_images, name="select_images"),
]
