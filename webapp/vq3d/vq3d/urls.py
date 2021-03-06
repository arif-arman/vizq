"""vq3d URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/1.10/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  url(r'^$', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  url(r'^$', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.conf.urls import url, include
    2. Add a URL to urlpatterns:  url(r'^blog/', include('blog.urls'))
"""
from django.conf.urls import url
from django.contrib import admin

from vq3d_app import views

urlpatterns = [
    url(r'^admin/', admin.site.urls),
    url(r'^mvq', views.mvq, name='mvq'),
    url(r'^vcm', views.vcm, name='vcm'),
    url(r'^cmvq', views.cmvq, name='cmvq'),
    url(r'^$', views.index, name='index'),
    url(r'^read_file', views.read_file, name='read_file'),
    url(r'^upload_qp', views.upload_qp, name='upload_qp'),
    url(r'^upload_ob', views.upload_ob, name='upload_ob'),
    url(r'^upload_tp', views.upload_tp, name='upload_tp'),
]
