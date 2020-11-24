FROM claudiaalvarezaparicio/brittany:github

USER root

RUN usermod -l jovyan student
RUN usermod -d /home/jovyan -m jovyan

USER jovyan

EXPOSE 8888
