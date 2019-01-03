#' ---
#' title: "Analyzing the performance of the OpenPlant incubator at Genspace"
#'output:
#'  html_document:
#'    toc: true
#'    theme: united
#' ---

#/*making a note book with knitr.spin is preferable because of the dynamic possible code debugging*/
#/* YAML header, #' for drop into Rmarkdown (#, ## headers), #+ for chunks (try not to disrupt code chunks with Rmarkdown, place before)*/

#+ intial-setup, message=FALSE, include=FALSE, echo=FALSE
#set global knitr options
knitr::opts_chunk$set(warning = FALSE, tidy = FALSE)

#tidyverse
require(readr)
require(dplyr)
require(tidyr)
require(stringr)
require(ggplot2)
# for time series data
require(zoo)

#+ session-info
sessionInfo() #for reproducibility

#' #Read in data and preprocess
#'
#' Declare the labels for the columns and list the files which will be read into the data table. Then prep the master data table as tidy data.
#+ import-process-data, message=FALSE
labels <- c("time", "temp", "humidity", "lux", "input", "fan", "output") #comment out this line for the updated code since fan and input are removed from the code
labels <- c("time", "temp", "humidity", "lux", "output")
setwd("BowB")
files <- c("180723.LOG",
           "180724.LOG",
           "180725.LOG",
           "180726.LOG",
           "180727.LOG",
           "180728.LOG",
           "180729.LOG",
           "180730.LOG",
           "180731.LOG",
           "180801.LOG",
           "180802.LOG",
           "180803.LOG",
           "180804.LOG",
           "180805.LOG",
           "180806.LOG",
           "180807.LOG",
           "180808.LOG",
           "180809.LOG",
           "180810.LOG",
           "180812.LOG",
           "180813.LOG",
           "180814.LOG",
           "180815.LOG",
           "180816.LOG",
           "180817.LOG",
           "180818.LOG",
           "180819.LOG")

working_data <- lapply(files, read_csv, col_names = labels) %>%
  bind_rows() %>%
  transform(time = as.POSIXct(strptime(time, "%Y%m%d_%H:%M:%S"))) %>%
#  select(-input, -fan) %>% #comment out this line for the updated code since fan and input are removed from the code
  gather(measure, value, -time) %>%
  arrange(time, measure)

#' #Visualze the data
#' 
#' plot the measures
#+ display-data
ggplot(data = working_data, aes(x = time, y = value)) +
  #scale_color_brewer(palette = "Set1") +
  geom_line() +
  #geom_line(aes(alpha = 0.5)) +
  #geom_line(aes(x = rollmean(time, 3, fill = NA), y = rollmean(value, 3, fill = NA), colour = measure)) +
  facet_grid(measure ~ ., scales = "free_y")

ggplot(data = filter(working_data, format(time, "%m%d") == "0818"), aes(x = time, y = value)) +
  geom_line() +
  facet_grid(measure ~ ., scales = "free_y")

#' zoom into the interesting features on July 7
