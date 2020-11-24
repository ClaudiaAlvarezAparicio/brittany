FROM claudiaalvarezaparicio/brittany:github

ARG NB_USER=jovyan
ARG NB_UID=1000
ENV USER ${NB_USER}
ENV NB_UID ${NB_UID}
ENV HOME /home/${NB_USER}

USER root

RUN usermod -l jovyan student
RUN usermod -d /home/jovyan -m jovyan



USER jovyan

EXPOSE 8888
