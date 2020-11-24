FROM claudiaalvarezaparicio/brittany:github

#RUN pip install --no-cache-dir notebook==5.*

ARG NB_USER=jovyan
ARG NB_UID=1000
ENV USER ${NB_USER}
ENV NB_UID ${NB_UID}
ENV HOME /home/${NB_USER}

USER root

RUN usermod -l jovyan student
RUN usermod -d /home/jovyan -m jovyan

RUN chown -R ${NB_UID} ${HOME}
USER ${NB_USER}

USER jovyan

RUN ls -la /home/jovyan/
RUN cat /home/jovyan/get_data_rosbags.sh

EXPOSE 8888

ENTRYPOINT ["/home/jovyan/get_data_rosbags.sh"]
CMD ["start"]
