FROM espressif/idf:release-v5.4

ARG TZ
ENV TZ="$TZ"
ENV USERNAME=ubuntu

# -------------------------------
# Install Node.js 20 and packages
# -------------------------------
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt update && \
    apt install -y \
      nodejs \
      less \
      git \
      procps \
      sudo \
      zsh \
      man-db \
      unzip \
      gnupg2 \
      gh \
      iptables \
      ipset \
      iproute2 \
      dnsutils \
      aggregate \
      jq \
      libusb-1.0-0 \
      cmake \
      build-essential \
      python3 \
      python3-pip \
      python3-venv \
      locales \
      flex \
      bison \
      gperf \
      ccache \
      ninja-build \
      wget \
      curl

# -------------------------------
# Set up locale
# -------------------------------
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    locale-gen
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# -------------------------------
# Setup workspace and bash history
# -------------------------------
RUN mkdir -p /workspace /commandhistory /home/$USERNAME/.claude /usr/local/share/npm-global && \
    chown -R $USERNAME:$USERNAME /workspace /commandhistory /home/$USERNAME/.claude /usr/local/share/npm-global

RUN echo "export PROMPT_COMMAND='history -a'" >> /etc/bash.bashrc && \
    echo "export HISTFILE=/commandhistory/.bash_history" >> /etc/bash.bashrc && \
    touch /commandhistory/.bash_history && \
    chown $USERNAME:$USERNAME /commandhistory/.bash_history

# -------------------------------
# Install git-delta
# -------------------------------
WORKDIR /workspace
RUN ARCH=$(dpkg --print-architecture) && \
    wget "https://github.com/dandavison/delta/releases/download/0.18.2/git-delta_0.18.2_${ARCH}.deb" && \
    dpkg -i "git-delta_0.18.2_${ARCH}.deb" && \
    rm "git-delta_0.18.2_${ARCH}.deb"

# -------------------------------
# Set up user environment
# -------------------------------
USER $USERNAME

ENV NPM_CONFIG_PREFIX=/usr/local/share/npm-global
ENV PATH=$PATH:/usr/local/share/npm-global/bin
ENV SHELL=/bin/zsh
ENV DEVCONTAINER=true

# -------------------------------
# Powerline Zsh setup
# -------------------------------
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.0/zsh-in-docker.sh)" -- \
  -p git \
  -a "export PROMPT_COMMAND='history -a' && export HISTFILE=/commandhistory/.bash_history" \
  -x

# -------------------------------
# ESP-IDF autoload
# -------------------------------
RUN echo 'source $IDF_PATH/export.sh > /dev/null 2>&1' >> /home/$USERNAME/.zshrc && \
    echo 'source $IDF_PATH/export.sh > /dev/null 2>&1' >> /home/$USERNAME/.bashrc

# -------------------------------
# Install Claude Code
# -------------------------------
RUN npm install -g @anthropic-ai/claude-code

# -------------------------------
# Firewall script setup
# -------------------------------
USER root
COPY init-firewall.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/init-firewall.sh && \
    echo "$USERNAME ALL=(root) NOPASSWD: /usr/local/bin/init-firewall.sh" > /etc/sudoers.d/node-firewall && \
    chmod 0440 /etc/sudoers.d/node-firewall

USER $USERNAME
WORKDIR /workspace


